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


def test_tracker_noise_configuration_is_wired_to_factor_dimensions():
    tracker_yaml = (ROOT / "src" / "tracker" / "config" / "tracker.yaml").read_text(
        encoding="utf-8"
    )
    target_tracker = (ROOT / "src" / "tracker" / "src" / "target_tracker.cpp").read_text(
        encoding="utf-8"
    )
    tracker_node = (ROOT / "src" / "tracker" / "src" / "tracker_node.cpp").read_text(
        encoding="utf-8"
    )

    assert "prior_sigma:" in tracker_yaml
    assert "geometry_sigma:" in tracker_yaml
    assert "translation_sigma:" in tracker_yaml
    assert "velocity_sigma:" in tracker_yaml
    assert "pixel_sigma:" in tracker_yaml
    assert '"prior_sigma"' in tracker_node
    assert '"geometry_sigma"' in tracker_node
    assert "band.process_noise_z" in target_tracker
    assert "noiseModel::Diagonal::Sigmas" in target_tracker


def test_filtered_back_message_does_not_advance_shared_timestamp():
    tracker_node = (ROOT / "src" / "tracker" / "src" / "tracker_node.cpp").read_text(
        encoding="utf-8"
    )

    filter_start = tracker_node.index("} else if (hasRecentFrontTarget(stamp))")
    filter_return = tracker_node.index("return;", filter_start)
    timestamp_assignment = tracker_node.index("last_processed_stamp_ = stamp;")
    assert timestamp_assignment > filter_return
