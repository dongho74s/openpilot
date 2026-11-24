from openpilot.selfdrive.carrot.tuning_advice import advise as A

SETTINGS = {"LateralTorqueCustom": 1, "LateralTorqueKpV": 160, "LateralTorqueKiV": 70, "LateralAccelLimit": 35,
            "LatJerkMax": 100, "LatTorqueLimitPFilter": 0}
METRICS = {"lat_minutes": 10.0}


def win(err, out=None, rel=0, jit=0.02, sat=0.0, kind="turn"):
  m = {"err": err, "rel": rel, "jit": jit, "sat": sat}
  if out is not None:
    m["laneOut"] = out
  return m


def windows(cand_err, n=5, noise_err=0.101, ref_err=0.1, label="LateralAccelLimit:35->40", **cand):
  out = []
  for k in range(n):
    e = cand_err[k] if isinstance(cand_err, list) else cand_err
    out.append({"kind": "turn", "ref": win(ref_err, 0.30), "noise": win(noise_err, 0.30), "cand": {label: win(e, cand.get("out", 0.30), cand.get("rel", 0))}})
  return out


def test_candidates_stay_in_range_and_need_custom_torque_for_p_and_i():
  c = A.candidate_list(SETTINGS, p_filter_available=True)
  assert ("LateralAccelLimit", 35, 40) in c and ("LateralAccelLimit", 35, 30) in c
  assert ("LatTorqueLimitPFilter", 0, 30) in c and ("LateralTorqueKpV", 160, 170) in c
  edge = dict(SETTINGS, LateralAccelLimit=30, LateralTorqueKpV=200, LatTorqueLimitPFilter=30)
  c = A.candidate_list(edge, p_filter_available=False)
  assert ("LateralAccelLimit", 30, 25) not in c and ("LateralTorqueKpV", 200, 210) not in c
  assert not [x for x in c if x[0] == "LatTorqueLimitPFilter"]
  c = A.candidate_list(dict(SETTINGS, LateralTorqueCustom=0), p_filter_available=True)
  assert not [x for x in c if x[0] in ("LateralTorqueKpV", "LateralTorqueKiV")]


def test_a_consistent_gain_is_recommended():
  agg = A.aggregate(windows(0.08))                    # 20 % less error in every curve, noise 1 %
  rec = A.recommend(METRICS, agg, SETTINGS)
  assert rec["status"] == "change" and (rec["param"], rec["from"], rec["to"]) == ("LateralAccelLimit", 35, 40)
  assert rec["delta"]["better"] == 5 and rec["delta"]["worse"] == 0


def test_a_gain_inside_the_replay_noise_is_not():
  agg = A.aggregate(windows(0.08, noise_err=0.115))   # the noise run alone moves the error 15 %
  assert A.recommend(METRICS, agg, SETTINGS)["status"] == "keep"
  assert A.table(agg, SETTINGS)[0]["rejected"][0] == "no_effect"


def test_one_lucky_curve_is_not_enough():
  agg = A.aggregate(windows([0.02, 0.1, 0.1, 0.1, 0.1]))
  row = A.table(agg, SETTINGS)[0]
  assert "inconsistent" in row["rejected"]
  assert A.recommend(METRICS, agg, SETTINGS)["status"] == "keep"


def test_more_torque_let_go_rejects_a_change():
  agg = A.aggregate(windows(0.08, rel=2))
  assert "releases" in A.table(agg, SETTINGS)[0]["rejected"]
  assert A.recommend(METRICS, agg, SETTINGS)["status"] == "keep"


def test_p_is_never_recommended_from_the_replay():
  agg = A.aggregate(windows(0.05, label="LateralTorqueKpV:160->170"))
  rows = A.table(agg, SETTINGS)
  assert rows[0]["reference_only"] and not rows[0]["rejected"]
  assert A.recommend(METRICS, agg, SETTINGS)["status"] == "keep"


def test_not_enough_data():
  assert A.recommend({"lat_minutes": 2.0}, None, SETTINGS)["status"] == "not_enough_data"
  agg = A.aggregate(windows(0.08, n=2))
  assert A.recommend(METRICS, agg, SETTINGS) ["reason"] == "curves"
  assert A.recommend(METRICS, None, SETTINGS, "plant") == {"status": "no_simulation", "reason": "plant", "candidates": [], "notes": []}


def test_change_review_compares_the_drive_before_a_single_change():
  before = {"route": "a", "settings": dict(SETTINGS, LateralTorqueKpV=150),
            "metrics": {"lat_minutes": 10, "err": 0.15, "takeovers_per_min": 3.0, "release_per_min": 1.0, "match_curve_pct": 80}}
  after = {"route": "b", "settings": SETTINGS,
           "metrics": {"lat_minutes": 10, "err": 0.11, "takeovers_per_min": 2.0, "release_per_min": 1.0, "match_curve_pct": 88}}
  r = A.change_review(after, [before])
  assert r["comparable"] and r["param"] == "LateralTorqueKpV" and (r["before"], r["after"]) == (150, 160)
  assert r["verdict"] == "better"
  worse = dict(after, metrics=dict(after["metrics"], err=0.2, takeovers_per_min=4.5, match_curve_pct=70))
  assert A.change_review(worse, [before])["verdict"] == "worse"
  two = dict(before, settings=dict(SETTINGS, LateralTorqueKpV=150, LateralAccelLimit=30))
  assert A.change_review(after, [two])["comparable"] is False
  assert A.change_review(after, [dict(before, settings=SETTINGS)]) is None


def test_change_review_counts_the_custom_torque_switch():
  before = {"route": "a", "settings": dict(SETTINGS, LateralTorqueCustom=0, LateralTorqueKiV=90), "metrics": {"lat_minutes": 10}}
  after = {"route": "b", "settings": SETTINGS, "metrics": {"lat_minutes": 10}}
  r = A.change_review(after, [before])
  assert r["comparable"] is False and set(r["changed"]) == {"LateralTorqueCustom", "LateralTorqueKiV"}
  before = {"route": "a", "settings": dict(SETTINGS, LateralTorqueCustom=0), "metrics": {"lat_minutes": 10}}
  r = A.change_review(after, [before])
  assert r["param"] == "LateralTorqueCustom" and (r["before"], r["after"]) == (0, 1)
  off = dict(SETTINGS, LateralTorqueCustom=0)
  r = A.change_review({"route": "b", "settings": off, "metrics": {}}, [{"route": "a", "settings": dict(off, LateralTorqueKiV=90), "metrics": {}}])
  assert r is None                                 # stored I does nothing with custom torque off on both drives


def test_every_tuning_value_gets_its_one_step_candidates():
  full = A.drive_settings({"LateralTorqueCustom": "1", "LateralTorqueKpV": "160", "LateralTorqueKiV": "50", "LateralTorqueKf": "115",
                           "LateralAccelLimit": "35", "LatJerkMax": "100", "CustomSteerDeltaUp": "0", "CustomSteerDeltaDown": "0"})
  limits = {"CustomSteerDeltaUp": (10, 12), "CustomSteerDeltaDown": (15, 25)}
  c = A.candidate_list(full, p_filter_available=True, torque_limits=limits)
  assert ("LateralTorqueKf", 115, 125) in c and ("LateralTorqueKf", 115, 105) in c
  assert ("LatJerkMax", 100, 90) in c and ("LatJerkMax", 100, 110) not in c          # 100 is the top of the range
  # 0 = the car's own (10 up, 15 down): one step from there, never past the panda limit (12 up)
  assert ("CustomSteerDeltaUp", 0, 12) in c and ("CustomSteerDeltaUp", 0, 8) in c
  assert ("CustomSteerDeltaDown", 0, 18) in c and ("CustomSteerDeltaDown", 0, 12) in c
  up = A.candidate_list(dict(full, CustomSteerDeltaUp=12), True, limits)
  assert [x for x in up if x[0] == "CustomSteerDeltaUp"] == [("CustomSteerDeltaUp", 12, 10)]
  assert ("LateralTorqueKd", 0, 10) in c
  assert not [x for x in A.candidate_list(full, True) if x[0] in A.DELTA_KEYS]           # no limits known: skipped
  off = A.candidate_list(dict(full, LateralTorqueCustom=0), True, limits)
  assert not [x for x in off if x[0] in A.CUSTOM_ONLY]


def test_kd_is_reference_only_like_p():
  agg = A.aggregate(windows(0.05, label="LateralTorqueKd:0->10"))
  row = A.table(agg, SETTINGS)[0]
  assert row["reference_only"] and A.recommend(METRICS, agg, SETTINGS)["status"] == "keep"


def test_change_review_ignores_keys_an_older_result_did_not_have():
  old = {"route": "a", "settings": dict(SETTINGS, LateralTorqueKiV=70), "metrics": {"lat_minutes": 10}}   # 6 keys only
  new = {"route": "b", "settings": A.drive_settings({k: str(v) for k, v in dict(SETTINGS, LateralTorqueKiV=50).items()}),
         "metrics": {"lat_minutes": 10}}
  r = A.change_review(new, [old])
  assert r["comparable"] and r["param"] == "LateralTorqueKiV"


def test_every_setting_the_advice_reads_comes_from_the_drive():
  from openpilot.selfdrive.carrot.tuning_advice.extract import TUNING_PARAMS
  assert set(A.SETTING_DEFAULTS) <= set(TUNING_PARAMS)


def test_a_review_against_an_older_format_is_not_judged():
  old = {"route": "a", "settings": dict(SETTINGS, LateralTorqueKpV=150),
         "metrics": {"lat_minutes": 10, "err": 0.15, "takeovers_per_min": 3.0, "release_per_min": 1.0, "match_curve_pct": 80}}
  new = {"route": "b", "settings": A.drive_settings({k: str(v) for k, v in SETTINGS.items()}),
         "metrics": {"lat_minutes": 10, "err": 0.11, "takeovers_per_min": 2.0, "release_per_min": 1.0, "match_curve_pct": 88}}
  r = A.change_review(new, [old])
  assert r["param"] == "LateralTorqueKpV" and r["better"] >= 2 and r["verdict"] == "unclear"
  assert "LateralTorqueKf" in r["missing_keys"] and "CustomSteerMax" in r["missing_keys"]
