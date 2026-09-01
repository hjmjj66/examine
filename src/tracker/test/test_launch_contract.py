from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_bringup_starts_tracker_and_preserves_downstream_contracts():
    bringup = (ROOT / "src" / "sentry_tf" / "launch" / "sentry_bringup.launch.py").read_text(
        encoding="utf-8"
    )
    start_script = (ROOT / "start.bash").read_text(encoding="utf-8")

    assert 'get_package_share_directory("tracker")' in bringup
    assert 'include_launch(\n            "tracker",\n            "tracker.launch.py"' in bringup
    assert 'include_launch(\n            "aim_predictor"' not in bringup
    assert '"/aim_predictor/fused/target_states"' in (
        ROOT / "src" / "tracker" / "config" / "tracker.yaml"
    ).read_text(encoding="utf-8")
    assert '"aim_outpost_predictor"' in bringup
    assert '"aim_armor_decider"' in bringup
    assert '"aim_armor_controller"' in bringup
    assert 'launch_bg "tracker" ros2 launch tracker tracker.launch.py' in start_script
    assert 'launch_bg "aim_predictor"' not in start_script
