import pytest
import requests


@pytest.fixture(autouse=True)
def no_external_http(monkeypatch):
  def forbidden(*args, **kwargs):
    raise AssertionError("Tests must not contact a real Cloud/vehicle endpoint")
  monkeypatch.setattr(requests.sessions.Session, "request", forbidden)
