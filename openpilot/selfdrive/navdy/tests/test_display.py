import copy
import json
import math
from types import SimpleNamespace as NS
import pytest

from openpilot.selfdrive.navdy.display import (
  HIDDEN_BAND, MAX_PROJECTED_POINTS, _model_points, lateral_scale, model_line_y_at_values, navdy_model_geometry,
  navdy_model_line, navdy_project_point, payload_from_messages,
)


def car(**kwargs):
  values = {"canValid": True, "canTimeout": False, "vEgo": 20.0, "vEgoCluster": 21.0,
            "vCruise": 90.0, "vCruiseCluster": 91.0, "gearShifter": "drive", "doorOpen": False,
            "standstill": False, "leftBlinker": True, "rightBlinker": False,
            "leftBlindspot": False, "rightBlindspot": True, "softHoldActive": 0,
            "brakeHoldActive": False, "cruiseState": NS(speed=25.0, speedCluster=26.0)}
  return NS(**(values | kwargs))


def selfdrive(**kwargs):
  values = {"state": "enabled", "enabled": True, "active": True, "engageable": True,
            "alertText1": "", "alertText2": "", "alertType": "", "alertStatus": "normal", "alertSize": "none"}
  return NS(**(values | kwargs))


def model(samples=33):
  xs = [80.0 * i / (samples - 1) for i in range(samples)]
  def line(y):
    return NS(x=xs[:], y=[y] * samples)
  return NS(position=line(0), laneLines=[line(-5.4), line(-1.8), line(1.8), line(5.4)],
            laneLineProbs=[0.8, 0.9, 0.95, 0.7], roadEdges=[line(-7), line(7)], roadEdgeStds=[0.1, 0.8])


def lead(**kwargs):
  values = {"status": True, "dRel": 40.0, "yRel": 0.0, "vRel": -2.0, "radar": True, "radarTrackId": 42, "modelProb": 0.9}
  return NS(**(values | kwargs))


def payload(**kwargs):
  return payload_from_messages(**({"car_state": car(), "selfdrive_state": selfdrive(), "onroad": True} | kwargs))


class TestDisplay:
  def test_golden_payload(self):
    actual = payload(seq=12)
    assert actual == {
      "schema": "navdy.openpilot.v1", "seq": 12, "state": "enabled",
      "enabled": True, "active": True, "engaged": True, "disengaged": False,
      "engageable": True, "opAvailable": True, "onroad": True, "doorOpen": False,
      "ambientState": "onroad", "gear": "drive", "standstill": False,
      "cruiseStandstill": False, "autoHoldActive": False, "softHoldActive": 0,
      "setSpeedKph": 91.0, "actualAccSetKph": 93.6, "vEgoKph": 75.6, "speedUnit": "kph",
      "leftBlinker": True, "rightBlinker": False, "blinkers": "left",
      "leftBlindspot": False, "rightBlindspot": True, "blindspot": "right",
      "alertText1": "", "alertText2": "", "alertType": "", "alertStatus": "normal",
      "alertSize": "none", "greenLightAlert": False, "leadDepartAlert": False,
      "longitudinalActuator": "none", "longitudinalActuatorLevel": 0.0,
      "latActive": False, "longActive": False, "activeLaneLine": False, "navVehicles": [], "decel": 0,
    }

  def test_speed_units_and_fallbacks(self):
    for cs, cc, speed in [
      (car(vCruiseCluster=0), None, 90.0),
      (car(vCruiseCluster=255, vCruise=255), None, 93.6),
      (car(vCruiseCluster=0, vCruise=0, cruiseState=NS(speed=10, speedCluster=0)), None, 36.0),
      (car(vCruiseCluster=0, vCruise=0, cruiseState=None), NS(hudControl=NS(setSpeed=10)), 36.0),
      (car(vCruiseCluster=math.nan, vCruise=math.inf, cruiseState=None), None, 0.0),
    ]:
      assert payload(car_state=cs, car_control=cc)["setSpeedKph"] == speed
    assert payload(car_state=car(vEgoCluster=0))["vEgoKph"] == 72.0
    assert payload(car_state=car(vEgo=10, vEgoCluster=0))["vEgoKph"] == 36.0
    assert payload(controls_state=NS(vCruise=150), longitudinal_plan=NS(cruiseTarget=140),
                   carrot_man=NS(desiredSpeed=130))["setSpeedKph"] == 91.0

  @pytest.mark.parametrize("gear", ["drive", "neutral", "park", "reverse"])
  def test_signed_speed_does_not_invalidate_fresh_vehicle_state(self, gear):
    # GM supplies signed wheel speed; the filter also crosses zero after stopping.
    speeds = (0.018, -0.01721164584, 0.0010106927, -0.000053814, 2.313e-6,
              -3.7476e-17, 2.34338e-18, -3.39526e-22, -1.22067e-41, 0.0)
    reference = payload(car_state=car(gearShifter=gear, vEgo=0, vEgoCluster=0, standstill=True))
    for speed in speeds:
      result = payload(car_state=car(gearShifter=gear, vEgo=speed, vEgoCluster=0, standstill=True))
      assert result["gear"] == gear
      for key in ("enabled", "active", "engageable", "opAvailable", "ambientState", "setSpeedKph", "actualAccSetKph"):
        assert result[key] == reference[key], key
      assert result["vEgoKph"] >= 0

  def test_reverse_signed_speed_never_grants_active_visuals(self):
    for speed in (-1.2078565, -0.018, 0, 0.018):
      result = payload(car_state=car(gearShifter="reverse", vEgo=speed), model=model())
      assert result["gear"] == "reverse"
      assert result["ambientState"] == "reverse"
      assert not result["active"] and not result["enabled"]
      assert result["navVehicles"] == [] and "navPathLeft" not in result

  @pytest.mark.parametrize("speed", [math.nan, math.inf, -math.inf, None, "bad", -100.0, 100.0, -101.0, 101.0])
  def test_invalid_speed_is_still_rejected(self, speed):
    result = payload(car_state=car(vEgo=speed), model=model())
    assert result["gear"] == "unknown"
    assert not result["active"] and not result["enabled"]
    assert result["ambientState"] == "disconnected"
    assert result["navVehicles"] == [] and "navPathLeft" not in result

  def test_original_alerts_are_never_suppressed_or_replaced(self):
    for event in ("resumeRequired", "brakeHold", "silentBrakeHold", "steerRequired"):
      for status, size in (("normal", "small"), ("userPrompt", "mid"), ("critical", "full")):
        sd = selfdrive(alertText1="Original \uacbd\uace0", alertText2="Keep this second line",
                       alertType=f"{event}/normal", alertStatus=status, alertSize=size)
        for onroad in (True, False):
          result = payload(selfdrive_state=sd, onroad=onroad)
          for field in ("alertText1", "alertText2", "alertType", "alertStatus", "alertSize"):
            assert result[field] == getattr(sd, field)

  def test_ambient_canonical_states(self):
    for onroad, door, gear, expected in [
      (False, False, "park", "offroad"), (False, True, "park", "offroad_door"),
      (True, False, "drive", "onroad"), (True, True, "drive", "onroad_door"),
      (True, True, "reverse", "reverse"), (False, False, "reverse", "reverse"),
    ]:
      assert payload(car_state=car(gearShifter=gear), onroad=onroad, door_open=door)["ambientState"] == expected
    assert payload(car_state=None, door_open=True)["ambientState"] == "disconnected"
    assert payload(selfdrive_state=None)["ambientState"] == "disconnected"
    assert payload(onroad=False, selfdrive_state=None)["ambientState"] == "offroad"
    assert payload(car_state=NS())["ambientState"] == "disconnected"

  def test_offroad_invalid_and_disengaged_have_no_active_visuals(self):
    for overrides in ({"onroad": False}, {"car_state": None}, {"selfdrive_state": None},
                      {"car_state": car(canValid=False)}, {"car_state": car(canTimeout=True)},
                      {"car_state": car(vEgo=math.nan)},
                      {"car_state": car(gearShifter="reverse")}):
      result = payload(model=model(), radar_state=NS(leadOne=lead()), **overrides)
      assert not result["active"]
      assert not result["engaged"]
      assert result["navVehicles"] == []
      assert "navPathLeft" not in result and "roadPreview" not in result
      assert result["longitudinalActuator"] == "none"
    # Disengaged with valid inputs (HUD v183): the road preview, never active visuals and never a visible path.
    for disengaged in (selfdrive(enabled=False, active=True), selfdrive(active=False)):
      result = payload(model=model(), radar_state=NS(leadOne=lead()), selfdrive_state=disengaged)
      assert not result["active"] and not result["engaged"] and result["longitudinalActuator"] == "none"
      assert result["roadPreview"] is True and len(result["navVehicles"]) == 1
      assert result["navPathLeft"] == result["navPathRight"] == [160.0, 1000.0, 160.0, 1000.0]
    empty = payload_from_messages()
    assert empty["ambientState"] == "disconnected"
    assert not empty["enabled"]

  def test_blinkers_bsm_and_hold(self):
    result = payload(car_state=car(rightBlinker=True, leftBlindspot=True, standstill=True, softHoldActive=2))
    assert result["blinkers"] == "hazard"
    assert result["blindspot"] == "both"
    assert result["autoHoldActive"]
    assert result["standstill"]
    assert not payload(car_state=car(softHoldActive=1))["autoHoldActive"]
    assert payload(car_state=car(brakeHoldActive=True, standstill=True),
                   selfdrive_state=selfdrive(enabled=False, active=False))["standstill"]
    assert payload(selfdrive_state=selfdrive(state="preEnabled", active=False))["standstill"]
    assert not payload(car_state=car(standstill=True),
                       selfdrive_state=selfdrive(enabled=False, active=False))["standstill"]

  def test_actuator_uses_returned_si_value_and_long_active(self):
    for accel, state, level in ((0.6, "accelerator", 0.6), (-0.8, "brake", 0.8),
                              (0.01, "coast", 0.0), (-4, "brake", 1.0), (math.nan, "none", 0.0)):
      result = payload(car_control=NS(longActive=True, latActive=True, actuators=NS(accel=3)),
                       car_output=NS(actuatorsOutput=NS(accel=accel)))
      assert (result["longitudinalActuator"], result["longitudinalActuatorLevel"]) == (state, level)
    assert payload(car_control=NS(longActive=False),
                   car_output=NS(actuatorsOutput=NS(accel=1)))["longitudinalActuator"] == "none"

  def test_projection_golden_wayon(self):
    # The v180-v182 road drawing (restored 2026-11-15).
    assert navdy_project_point(0, -1) == [116.0, 96.0]
    assert navdy_project_point(80, 1) == [168.0, 8.0]
    assert navdy_project_point(80, 3, 2) == [202.0, 8.0]
    assert navdy_project_point(40, 0) == [160.0, 39.9]
    assert navdy_model_line([0, 80], [0, 0], -1) == [116.0, 96.0, 152.0, 8.0]
    assert [lateral_scale(x) for x in (0.0, 80.0)] == [44.0, 8.0]
    assert model_line_y_at_values([0, 40, 80], [0, 2, 4], 20) == 1.0
    assert model_line_y_at_values([0, 40], [0, 2], 80) is None
    assert model_line_y_at_values([0, 60, 90], [0, 0, 3], 75) == 1.5

  def test_geometry_probabilities_edges_and_bounds(self):
    geometry = navdy_model_geometry(model())
    assert geometry["navPathLeft"][:2] == [116.0, 96.0]
    assert geometry["navLaneLeft"][-2:] == [145.6, 8.0]
    assert geometry["navLaneRightProb"] == 0.95
    assert geometry["navRoadEdgeLeftProb"] == 0.9
    assert "navRoadEdgeRight" not in geometry
    for value in geometry.values():
      if isinstance(value, list):
        assert len(value) <= MAX_PROJECTED_POINTS * 2
    assert navdy_model_geometry(None) == {}
    broken = model()
    broken.position.y = [math.nan] * 33
    assert navdy_model_geometry(broken) == {}
    broken = model()
    broken.roadEdgeStds = [-1, math.nan]
    assert "navRoadEdgeLeft" not in navdy_model_geometry(broken)
    assert "navRoadEdgeRight" not in navdy_model_geometry(broken)

  @pytest.mark.parametrize("tail", [21.8, 22.0])
  def test_folded_or_stopped_path_keeps_only_current_forward_prefix(self, tail):
    m = model()
    m.position = NS(x=[0., 5., 15., 22., tail, 30.], y=[0., 0.1, 0.3, 0.5, 9., 12.])
    expected = copy.deepcopy(m)
    expected.position.x = expected.position.x[:4]
    expected.position.y = expected.position.y[:4]
    geometry = navdy_model_geometry(m)
    assert geometry and geometry == navdy_model_geometry(expected)
    assert model_line_y_at_values(m.position.x, m.position.y, 10.) == pytest.approx(.2)
    assert model_line_y_at_values(m.position.x, m.position.y, 25.) is None
    assert m.position.x == [0., 5., 15., 22., tail, 30.]
    assert m.position.y[-2:] == [9., 12.]

  def test_degenerate_prefix_does_not_rejoin_later_forward_points(self):
    for xs in ([0., 0., 20.], [20., 10., 30.], [0.]):
      assert navdy_model_line(xs, [0.] * len(xs)) == []
      assert model_line_y_at_values(xs, [0.] * len(xs), 15.) is None

  @pytest.mark.parametrize("origin", [0., .00005])
  def test_negative_submillimeter_origin_jitter_preserves_current_forward_path(self, origin):
    xs, ys = [0., origin, -.00055, .002, 10.], [0., 0., -.000007, .00001, .2]
    if origin == 0.:
      xs, ys = xs[1:], ys[1:]
    kept = [(x, y) for x, y in zip(xs, ys, strict=True) if x >= 0.]
    assert navdy_model_line(xs, ys) == navdy_model_line([p[0] for p in kept], [p[1] for p in kept])
    assert model_line_y_at_values(xs, ys, 5.) is not None

  @pytest.mark.parametrize("first", [-4.261169195274306e-11, -5.339151343264348e-11, -2.0650148258027912e-14, -.0009])
  def test_first_sample_float_noise_below_zero_keeps_the_road(self, first):
    # RDF plans (2026-11-15 drive) start at +-1e-10 m; a negative first sample emptied the path, so every such frame
    # sent no road, lanes or vehicles and the HUD flickered.
    m = model()
    m.position.x = [first] + list(m.position.x[1:])
    clean = model()
    clean.position.x = [0.] + list(clean.position.x[1:])
    geometry = navdy_model_geometry(m)
    assert geometry and geometry == navdy_model_geometry(clean)
    assert model_line_y_at_values(m.position.x, m.position.y, 5.) == model_line_y_at_values(clean.position.x, clean.position.y, 5.)
    cars = payload(model=m, radar_state=NS(leadOne=lead(dRel=30., yRel=0.)))
    assert cars['navPathLeft'] and len(cars['navVehicles']) == 1
    assert cars == payload(model=clean, radar_state=NS(leadOne=lead(dRel=30., yRel=0.)))

  @pytest.mark.parametrize("xs", [
    # stopped RDF plans of 2026-11-15 (frames 12005 and 13202): the car, then millimetres to centimetres behind it
    [-2.26e-11, -.00011, -.000526, -.00169, -.00457, -.00894, -.0102, -.017, -.0169, -.0211, -.0247, -.0257],
    [-1.69e-11, -4.21e-05, -.000211, -.000287, -.000676, -.00161, .000761, -.00242, .000496, -.00174, -.00125, .00175],
    [0.] * 12,
  ])
  def test_stopped_plan_keeps_lanes_and_vehicles_with_the_path_hidden(self, xs):
    m = model()
    m.position = NS(x=xs, y=[0.] * len(xs))
    road = payload(model=m, radar_state=NS(leadOne=lead(dRel=12., yRel=0.)))
    assert road['navPathLeft'] == list(HIDDEN_BAND) and road['navPathRight'] == list(HIDDEN_BAND)
    assert road['navLaneLeft'] == navdy_model_geometry(model())['navLaneLeft']   # no path: lanes uncorrected, as at 0
    assert len(road['navVehicles']) == 1
    for bad in ([math.nan] + xs[1:], [.002] + xs[1:], [-.002] + xs[1:]):
      m.position = NS(x=bad, y=[0.] * len(xs))
      assert navdy_model_geometry(m) == {}             # a plan not starting at the car is still no road

  @pytest.mark.parametrize("xs,ys", [                  # broken plans (Codex review 2026-11-15): still no road
    ([0., math.nan, 10.], [0., 0., 0.]),
    ([0., 1., 10.], [0., math.nan, 0.]),
    ([0., -100., -200.], [0., 0., 0.]),
    ([0., -.002, -.003], [50., 50., 50.]),
    ([0., -1.5, 2.], [0., 0., 0.]),
    ([0., -.01, -.02], [0., 0., 101.]),
    ([0., -.01], [0.]),
  ])
  def test_broken_plan_is_not_a_stopped_plan(self, xs, ys):
    m = model()
    m.position = NS(x=xs, y=ys)
    assert navdy_model_geometry(m) == {}
    assert payload(model=m, radar_state=NS(leadOne=lead(dRel=12., yRel=0.)))['navVehicles'] == []

  def test_stopped_plan_with_a_short_forward_prefix_still_draws_it(self):
    m = model()
    xs = [-1.64e-11, -8.11e-06, -5.13e-05, 8.16e-06, -.000155, -.000803, .0021, -.000587] + [.003] * 4   # frame 13202+
    m.position = NS(x=xs, y=[0.] * len(xs))
    road = payload(model=m, radar_state=NS(leadOne=lead(dRel=12., yRel=0.)))
    assert road['navPathLeft'] != list(HIDDEN_BAND) and len(road['navPathLeft']) >= 4
    assert road['navLaneLeft'] and len(road['navVehicles']) == 1

  @pytest.mark.parametrize("xs", [[-.002, 5., 10.], [-.0005, 0., 10.], [-1e-10, -2e-10, 10.]])
  def test_first_sample_noise_rule_stays_narrow(self, xs):
    ys = [0., 0., .2]
    points = _model_points(xs, ys)
    if xs[0] < -.001:
      assert points == []                            # a real negative start is still invalid
    elif xs[1] == 0.:
      assert points == []                            # the origin twice is not a forward line
    else:
      assert points == [(0., 0.), (10., .2)]         # standstill jitter after the origin is skipped as before

  @pytest.mark.parametrize("xs,ys", [
    ([0., -.002, 10.], [0., 0., 0.]),
    ([0., -.0005, 10.], [0., .002, 0.]),
    ([0., -.0005, 10.], [0., math.nan, 0.]),
    ([0., 1., -.0005, 10.], [0.] * 4),
  ])
  def test_origin_jitter_exception_never_bridges_a_real_gap(self, xs, ys):
    assert model_line_y_at_values(xs, ys, 5.) is None

  @pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, 101., -101.])
  def test_invalid_sample_never_joins_disconnected_valid_segments(self, bad):
    xs, ys = [0., 5., 10., 15.], [0., 1., bad, 3.]
    assert navdy_model_line(xs, ys) == navdy_model_line(xs[:2], ys[:2])
    assert model_line_y_at_values(xs, ys, 2.5) == .5
    assert model_line_y_at_values(xs, ys, 12.) is None
    assert model_line_y_at_values([0., 5., math.nan, 15.], [0.] * 4, 12.) is None

  @pytest.mark.parametrize("distance", [10., 30., 51., 52., 70., 78.])  # camera x <= the 80 m test lanes
  def test_lead_beyond_stopping_path_uses_same_projection_as_current_lanes(self, distance):
    m = model()
    m.position = NS(x=[0., 2., 5.], y=[0., 0., .1])
    for lane_line in m.laneLines:
      lane_line.y = [y + 1. for y in lane_line.y]
    cars = payload(model=m, radar_state=NS(leadOne=lead(dRel=distance, yRel=-1.)))['navVehicles']
    assert len(cars) == 1 and cars[0]['lane'] == 'center'
    x = distance + 1.52  # radar dRel -> camera, beyond the 5 m plan: drawn like the lanes there (its end held)
    assert [cars[0]['screenX'], cars[0]['screenY']] == navdy_project_point(x, 1., .1)
    assert cars[0]['screenX'] == navdy_model_line([0., x], [1., 1.],
                                               path_x_values=m.position.x, path_y_values=m.position.y)[-2]
    assert cars[0]['pxPerM'] == round(lateral_scale(x), 2)
    m.laneLineProbs = [0.] * 4
    assert not payload(model=m, radar_state=NS(leadOne=lead(dRel=distance)))['navVehicles']

  def test_short_path_does_not_grant_leads_beyond_measured_lanes(self):
    m = model()
    m.position = NS(x=[0., 5.], y=[0., 0.])
    for lane_line in m.laneLines:
      lane_line.x = [0., 10.]
      lane_line.y = lane_line.y[:2]
    assert not payload(model=m, radar_state=NS(leadOne=lead(dRel=30.)))['navVehicles']

  def test_leads_only_visible_one_and_two_no_tracks_or_warnings(self):
    radar = NS(leadOne=lead(), leadTwo=lead(dRel=20, yRel=3.6, radarTrackId=43),
               leadsLeft=[lead()] * 100, leadCutInRisk=lead())
    result = payload(model=model(), radar_state=radar,
                     longitudinal_plan=NS(longitudinalPlanSource="lead0", fcw=True, trafficState=2))
    def view(distance, lateral):
      x = distance + 1.52   # the test path is straight at y = 0
      point = navdy_project_point(x, lateral, 0.0)
      return {"screenX": point[0], "screenY": point[1], "pxPerM": round(lateral_scale(x), 2)}
    assert result["navVehicles"] == [
      {"trackId": 42, **view(40.0, 0.0), "distanceM": 40.0,
       "relativeSpeedMps": -2.0, "lateralM": 0.0, "lane": "center", "source": "radar",
       "longitudinalLead": True, "confidence": 0.9, "colorRole": 1},
      {"trackId": 43, **view(20.0, -3.6), "distanceM": 20.0,
       "relativeSpeedMps": -2.0, "lateralM": -3.6, "lane": "left", "source": "radar",
       "longitudinalLead": False, "confidence": 0.9, "colorRole": 2},
    ]
    assert not result["greenLightAlert"]
    assert not result["leadDepartAlert"]
    for name in ("nTracks", "cutInRisk", "collisionWarning", "automaticAccActive", "sectionPhase"):
      assert name not in result
    for kwargs in ({"status": False}, {"dRel": math.nan}, {"dRel": 81}, {"yRel": math.inf}, {"vRel": math.nan}):
      assert payload(model=model(), radar_state=NS(leadOne=lead(**kwargs)))["navVehicles"] == []
    assert payload(model=None, radar_state=radar)["navVehicles"] == []
    assert len(payload(model=model(), radar_state=NS(leadOne=lead(), leadTwo=lead()))["navVehicles"]) == 1

  def test_finite_json_deterministic_and_input_unchanged(self):
    inputs = {"car_state": car(vEgoCluster=math.nan, vEgo=math.inf, vCruise=math.nan),
              "selfdrive_state": selfdrive(), "model": model(), "radar_state": NS(leadOne=lead()),
              "car_control": NS(longActive=True), "car_output": NS(actuatorsOutput=NS(accel=math.inf)),
              "onroad": True, "seq": math.inf}
    before = copy.deepcopy(inputs)
    first = payload_from_messages(**inputs)
    assert first == payload_from_messages(**inputs)
    assert inputs["model"] == before["model"]
    assert inputs["radar_state"] == before["radar_state"]
    assert first["vEgoKph"] == 0.0
    json.dumps(first, allow_nan=False)
    for bad in (math.nan, math.inf, -math.inf, None, "bad"):
      assert navdy_project_point(bad, 0) == []
      assert navdy_project_point(20, bad) == []
    assert navdy_model_line([20, 10], [0, 0]) == []

  def test_payload_size_and_bounded_input(self):
    m = model(10000)
    result = payload(model=m, radar_state=NS(leadOne=lead(), leadTwo=lead(radarTrackId=43)),
                     selfdrive_state=selfdrive(alertText1="\uacbd" * 1000, alertText2="\uace0" * 1000))
    assert len(json.dumps(result, ensure_ascii=True, allow_nan=False).encode()) < 32768
    assert len(json.dumps(result, ensure_ascii=False, allow_nan=False).encode()) < 32768
    for name, value in result.items():
      if name.startswith("nav") and isinstance(value, list):
        assert len(value) <= MAX_PROJECTED_POINTS * 2


  def test_actual_carrot_capnp_structs(self):
    pytest.importorskip("capnp")
    from openpilot.cereal import car as car_schema, custom, log
    cs = car_schema.CarState.new_message(canValid=True, vEgo=10, vEgoCluster=10,
                                         vCruise=80, vCruiseCluster=80, gearShifter="drive")
    sd = log.SelfdriveState.new_message(enabled=True, active=True, engageable=True, state="enabled")
    result = payload_from_messages(
      car_state=cs.as_reader(), selfdrive_state=sd.as_reader(),
      car_control=car_schema.CarControl.new_message().as_reader(),
      car_output=car_schema.CarOutput.new_message().as_reader(),
      controls_state=log.ControlsState.new_message().as_reader(),
      model=log.ModelDataV2.new_message().as_reader(), radar_state=log.RadarState.new_message().as_reader(),
      longitudinal_plan=log.LongitudinalPlan.new_message().as_reader(),
      carrot_man=custom.CarrotMan.new_message().as_reader(), onroad=True)
    assert (result["state"], result["vEgoKph"], result["setSpeedKph"], result["ambientState"]) == ("enabled", 36.0, 80.0, "onroad")
