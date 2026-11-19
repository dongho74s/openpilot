"""Carrot B1 WiFi endpoints: GET /api/b1/intent, POST /api/b1/bsm.

The dual-transport B1 board (CarrotB1_Dual.ino) joins the comma hotspot
and polls /api/b1/intent every 100 ms for the lane-change intent, and
POSTs blind-spot state to /api/b1/bsm. See the board file's header for
the full contract (op values, the Monitor's shared seq counter, the
350 ms lease).

Trust model (same as the board header note): WiFi commands are NOT
HMAC-signed like BLE. Trust comes from the hotspot being private
(password-protected). Anyone on the hotspot could send intents — which
is no wider than the web UI already served on :7000 to the same
network. The port-80 listener below therefore binds ONLY to the
hotspot address (192.168.43.1), never 0.0.0.0: a board can only reach
it by joining the comma hotspot, and the endpoint is unreachable from
any other interface.
"""
import asyncio
import json

from aiohttp import web

from openpilot.selfdrive.carrot.bluetooth.model import write_b1_bsm
from openpilot.selfdrive.carrot.server.features.bluetooth import B1

# Must match openpilot/system/ui/lib/wifi_manager.py::TETHERING_IP_ADDRESS
# and the board's COMMA_IP in CarrotB1_Dual.ino (Android tether default).
# Kept as a literal here so this module stays importable without the
# NetworkManager/DBus stack wifi_manager pulls in.
HOTSPOT_IP = "192.168.43.1"
HOTSPOT_HTTP_PORT = 80
BSM_MAX_BYTES = 1024


def _monitor(request):
  monitor = request.app.get(B1)
  if monitor is None:
    raise web.HTTPServiceUnavailable(text='b1 monitor not running')
  return monitor


async def intent(request):
  raw_ack = request.query.get('ack', '0')
  try:
    ack = int(raw_ack)
  except ValueError:
    raise web.HTTPBadRequest(text='ack must be an integer') from None
  if not 0 <= ack <= 65535:
    raise web.HTTPBadRequest(text='ack out of range') from None
  return web.json_response(_monitor(request).wifi_intent(ack))


async def bsm(request):
  if request.content_length is not None and request.content_length > BSM_MAX_BYTES:
    raise web.HTTPRequestEntityTooLarge(max_size=BSM_MAX_BYTES, actual_size=request.content_length)
  raw = await request.content.read(BSM_MAX_BYTES + 1)
  if len(raw) > BSM_MAX_BYTES:
    raise web.HTTPRequestEntityTooLarge(max_size=BSM_MAX_BYTES, actual_size=len(raw))
  try:
    body = json.loads(raw)
  except (ValueError, UnicodeDecodeError):
    raise web.HTTPBadRequest(text='invalid JSON') from None
  if not isinstance(body, dict) or type(body.get('left')) is not bool or type(body.get('right')) is not bool:
    raise web.HTTPBadRequest(text='left/right must be booleans') from None
  write_b1_bsm(body['left'], body['right'])
  return web.json_response({'ok': True})


def create_port80_app(monitor):
  """Minimal app for the hotspot-only :80 listener: B1 routes only.

  No static files, no other /api routes — a client on :80 can only poll
  intents and post blind-spot state. The app binds to HOTSPOT_IP only
  (see serve_port80); binding anything else would widen the trust zone.
  """
  app = web.Application()
  app[B1] = monitor
  app.router.add_get('/api/b1/intent', intent)
  app.router.add_post('/api/b1/bsm', bsm)
  return app


async def serve_port80(monitor):
  """Bind the B1-only app to the hotspot address, retrying until it sticks.

  The hotspot may come up after the server starts, so a failed bind is
  not fatal: log and retry. A bind failure never takes the :7000 web UI
  down — the board simply keeps working over BLE until WiFi is up.
  """
  while True:
    runner = web.AppRunner(create_port80_app(monitor))
    try:
      await runner.setup()
      await web.TCPSite(runner, HOTSPOT_IP, HOTSPOT_HTTP_PORT).start()
    except OSError as exc:
      await runner.cleanup()
      print(f"[b1_wifi] port {HOTSPOT_HTTP_PORT} on {HOTSPOT_IP} unavailable ({exc}); B1 WiFi disabled, retrying")
      await asyncio.sleep(30)
      continue
    print(f"[b1_wifi] serving /api/b1 on {HOTSPOT_IP}:{HOTSPOT_HTTP_PORT}")
    try:
      await asyncio.Event().wait()
    finally:
      await runner.cleanup()


def register(app):
  # Same routes on the main :7000 server (handy for debugging from a
  # phone on the hotspot); the board itself uses :80 (see serve_port80).
  app.router.add_get('/api/b1/intent', intent)
  app.router.add_post('/api/b1/bsm', bsm)

  async def _start_port80(app):
    monitor = app.get(B1)
    if monitor is None:
      return
    app['b1_port80_task'] = asyncio.create_task(serve_port80(monitor))

  async def _stop_port80(app):
    task = app.get('b1_port80_task')
    if task is not None:
      task.cancel()
      await asyncio.gather(task, return_exceptions=True)

  app.on_startup.append(_start_port80)
  app.on_cleanup.append(_stop_port80)
