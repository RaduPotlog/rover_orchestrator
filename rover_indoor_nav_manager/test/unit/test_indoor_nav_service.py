# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import pytest

from fakes import FakeLocalization, FakeMaps, FakeRobot, FakeSaver, RecordingObserver
from rover_indoor_nav_manager.application.indoor_nav_service import IndoorNavService
from rover_indoor_nav_manager.domain.model import DomainError, LocalizationMode, Pose2D


def make(names=(), active='', pose=None):
    maps = FakeMaps(names, active)
    loc = FakeLocalization()
    saver = FakeSaver()
    robot = FakeRobot(pose)
    obs = RecordingObserver()
    return IndoorNavService(maps, loc, saver, robot, obs), maps, loc, saver, robot, obs


def test_startup_without_maps_starts_mapping():
    svc, _, loc, _, _, obs = make()
    svc.startup()
    assert svc.state.mode == LocalizationMode.MAPPING
    assert loc.calls == [('stop',), ('mapping',)]
    assert [s.mode for s in obs.states] == [LocalizationMode.SWITCHING, LocalizationMode.MAPPING]


def test_startup_resumes_the_active_map_at_its_last_pose():
    svc, maps, loc, _, _, _ = make(names=['lab'], active='lab')
    maps.maps['lab']['pose'] = Pose2D(2, 3, 1)
    svc.startup()
    assert svc.state.mode == LocalizationMode.LOCALIZATION
    assert svc.state.map_name == 'lab'
    assert loc.calls[-1] == ('localization', '/maps/lab/map.yaml', Pose2D(2, 3, 1))


def test_startup_with_a_missing_active_map_falls_back_to_mapping():
    svc, _, _, _, _, _ = make(names=[], active='gone')
    svc.startup()
    assert svc.state.mode == LocalizationMode.MAPPING


def test_save_then_load_keeps_the_slam_pose():
    svc, maps, loc, saver, robot, obs = make(pose=Pose2D(4, 5, 0.5))
    svc.start_mapping()
    svc.save_map('lab')
    assert saver.saved == ['/maps/lab/map']
    assert maps.active == ''  # saving does not switch
    robot.pose = Pose2D(6, 7, 0.1)
    svc.load_map('lab')
    assert loc.calls[-1] == ('localization', '/maps/lab/map.yaml', Pose2D(6, 7, 0.1))
    assert maps.active == 'lab'
    assert obs.maps[-1] == (['lab'], 'lab')


def test_save_map_rules():
    svc, maps, _, saver, _, _ = make(names=['lab'])
    with pytest.raises(DomainError, match='Start mapping'):
        svc.save_map('new')
    svc.start_mapping()
    with pytest.raises(DomainError, match='already exists'):
        svc.save_map('lab')
    with pytest.raises(DomainError, match='Invalid map name'):
        svc.save_map('../x')
    saver.fail = True
    with pytest.raises(DomainError, match='Saving the map failed'):
        svc.save_map('hall')
    assert not maps.exists('hall')  # the half-written directory is removed


def test_load_map_with_an_explicit_pose_and_unknown_map():
    svc, _, loc, _, _, _ = make(names=['lab'])
    with pytest.raises(DomainError, match='No map'):
        svc.load_map('nope')
    svc.load_map('lab', Pose2D(1, 1, 0))
    assert loc.calls[-1][2] == Pose2D(1, 1, 0)


def test_load_without_any_pose_seeds_the_origin():
    svc, _, loc, _, _, _ = make(names=['lab'])
    svc.load_map('lab')
    assert loc.calls[-1][2] == Pose2D(0, 0, 0)


def test_delete_map_refuses_the_map_in_use():
    svc, maps, _, _, _, _ = make(names=['lab', 'hall'], active='lab')
    svc.load_map('lab')
    with pytest.raises(DomainError, match='in use'):
        svc.delete_map('lab')
    svc.delete_map('hall')
    assert not maps.exists('hall')


def test_deleting_the_remembered_active_map_clears_it():
    svc, maps, _, _, _, _ = make(names=['lab'], active='lab')
    svc.start_mapping()
    svc.delete_map('lab')
    assert maps.active == ''


def test_places_need_a_loaded_map_and_persist():
    svc, maps, _, _, _, obs = make(names=['lab'])
    with pytest.raises(DomainError, match='load one first'):
        svc.save_place('Dock', Pose2D(0, 0))
    svc.load_map('lab')
    dock = svc.save_place('Dock', Pose2D(1, 2, 0))
    assert [p.name for p in maps.maps['lab']['places']] == ['Dock']
    svc.save_place('Charger', Pose2D(1, 2, 0), dock.id)
    assert [p.name for p in svc.places()] == ['Charger']
    with pytest.raises(DomainError):
        svc.save_place('X', Pose2D(0, 0), 'unknown-id')
    svc.delete_place(dock.id)
    assert svc.places() == []
    assert obs.places[-1] == ('lab', [])


def test_switching_maps_swaps_the_place_book():
    svc, maps, _, _, _, _ = make(names=['lab', 'hall'])
    svc.load_map('lab')
    svc.save_place('Dock', Pose2D(0, 0))
    svc.load_map('hall')
    assert svc.places() == []
    svc.load_map('lab')
    assert [p.name for p in svc.places()] == ['Dock']


def test_record_pose_only_while_localized():
    svc, maps, _, _, robot, _ = make(names=['lab'], pose=Pose2D(3, 3, 0))
    svc.start_mapping()
    svc.record_pose()
    assert maps.maps['lab']['pose'] is None
    svc.load_map('lab')
    robot.pose = Pose2D(9, 9, 0)
    svc.record_pose()
    assert maps.maps['lab']['pose'] == Pose2D(9, 9, 0)


def test_health_check_reports_a_dead_stack_and_start_failure_is_reported():
    svc, _, loc, _, _, _ = make()
    svc.start_mapping()
    loc.alive = False
    svc.check_health()
    assert svc.state.mode == LocalizationMode.UNAVAILABLE
    assert 'unexpectedly' in svc.state.message
    loc.fail_next = True
    with pytest.raises(RuntimeError):
        svc.start_mapping()
    assert svc.state.mode == LocalizationMode.UNAVAILABLE
    assert 'Could not start SLAM' in svc.state.message


def test_stop_saves_the_pose_while_localized():
    svc, maps, _, _, robot, _ = make(names=['lab'], pose=Pose2D(1, 1, 0))
    svc.load_map('lab')
    robot.pose = Pose2D(4, 2, 0.3)
    svc.on_motion(0.5, 0.0, 0.0)
    svc.on_motion(0.0, 0.0, 1.0)
    assert maps.maps['lab']['pose'] != Pose2D(4, 2, 0.3)  # not yet: still for < hold time
    svc.on_motion(0.0, 0.0, 1.6)
    assert maps.maps['lab']['pose'] == Pose2D(4, 2, 0.3)


def test_stop_does_nothing_while_mapping():
    svc, maps, _, _, robot, _ = make(names=['lab'], pose=Pose2D(4, 2, 0.3))
    svc.start_mapping()
    for v, t in ((0.5, 0.0), (0.0, 1.0), (0.0, 2.0)):
        svc.on_motion(v, 0.0, t)
    assert maps.maps['lab']['pose'] is None


def test_shutdown_and_switching_away_save_the_pose_first():
    svc, maps, loc, _, robot, _ = make(names=['lab', 'hall'], pose=Pose2D(1, 1, 0))
    svc.load_map('lab')
    robot.pose = Pose2D(7, 7, 1.0)
    svc.load_map('hall')
    assert maps.maps['lab']['pose'] == Pose2D(7, 7, 1.0)
    robot.pose = Pose2D(3, 3, 0.0)
    svc.shutdown()
    assert maps.maps['hall']['pose'] == Pose2D(3, 3, 0.0)
    assert loc.calls[-1] == ('stop',)


def test_reloading_the_same_map_starts_from_the_freshest_pose():
    svc, maps, loc, _, robot, _ = make(names=['lab'], pose=Pose2D(1, 1, 0))
    svc.load_map('lab')
    robot.pose = Pose2D(5, 6, 0.2)
    svc.load_map('lab')
    assert loc.calls[-1] == ('switch', '/maps/lab/map.yaml', Pose2D(5, 6, 0.2))


def test_only_a_remembered_pose_gets_the_wider_spread():
    # Remembered pose (reboot): widened with the configured sigmas.
    svc, maps, loc, _, _, _ = make(names=['lab'], active='lab')
    maps.maps['lab']['pose'] = Pose2D(2, 3, 1)
    svc.startup()
    assert loc.widened and loc.widened[-1][0] == Pose2D(2, 3, 1)
    assert loc.widened[-1][1] == pytest.approx(1.5)
    assert 'remembered pose' in svc.state.message

    # Explicit pose: the operator knows where the rover is - AMCL's default spread.
    svc, _, loc, _, _, _ = make(names=['lab'])
    svc.load_map('lab', Pose2D(1, 1, 0))
    assert loc.widened == []

    # Nothing remembered: origin fallback, default spread (Set pose / Find me is needed anyway).
    svc, _, loc, _, _, _ = make(names=['lab'])
    svc.load_map('lab')
    assert loc.calls[-1][2] == Pose2D(0, 0, 0)
    assert loc.widened == []

    # SLAM hand-off: exact, never widened.
    svc, _, loc, _, robot, _ = make(pose=Pose2D(4, 5, 0.5))
    svc.start_mapping()
    svc.save_map('lab')
    svc.load_map('lab')
    assert loc.widened == []


def test_widen_failure_keeps_localization_and_says_so():
    svc, maps, loc, _, _, _ = make(names=['lab'], active='lab')
    maps.maps['lab']['pose'] = Pose2D(2, 3, 1)
    loc.widen_error = 'AMCL did not come up within 20 s'
    svc.startup()
    assert svc.state.mode == LocalizationMode.LOCALIZATION
    assert 'could not widen' in svc.state.message


def test_switching_saved_maps_while_localized_restarts_nothing():
    svc, maps, loc, _, robot, obs = make(names=['lab', 'hall'], pose=Pose2D(1, 1, 0))
    maps.maps['hall']['pose'] = Pose2D(8, 9, 0.5)
    svc.load_map('lab')
    svc.save_place('Dock', Pose2D(0, 0))
    del loc.calls[:]
    svc.load_map('hall')
    assert loc.calls == [('switch', '/maps/hall/map.yaml', Pose2D(8, 9, 0.5))]
    # A remembered pose still gets the wider search, as after a restart.
    assert loc.widened[-1][0] == Pose2D(8, 9, 0.5)
    assert svc.state.mode == LocalizationMode.LOCALIZATION
    assert svc.state.map_name == 'hall'
    assert maps.active == 'hall'
    assert svc.places() == []
    assert obs.maps[-1] == (['hall', 'lab'], 'hall')
    assert [s.mode for s in obs.states[-2:]] == [LocalizationMode.SWITCHING,
                                                  LocalizationMode.LOCALIZATION]


def test_failed_in_place_switch_falls_back_to_a_restart():
    svc, _, loc, _, _, _ = make(names=['lab', 'hall'])
    svc.load_map('lab')
    loc.switch_error = 'map_server did not answer'
    del loc.calls[:]
    svc.load_map('hall', Pose2D(2, 2, 0))
    assert loc.calls == [('stop',), ('localization', '/maps/hall/map.yaml', Pose2D(2, 2, 0))]
    assert svc.state.mode == LocalizationMode.LOCALIZATION
    assert svc.state.map_name == 'hall'
    assert 'in-place switch failed: map_server did not answer' in svc.state.message


@pytest.mark.parametrize('before', ['mapping', 'dead'])
def test_load_restarts_when_not_localizing(before):
    svc, _, loc, _, _, _ = make(names=['lab', 'hall'])
    if before == 'mapping':
        svc.start_mapping()
    else:
        svc.load_map('lab')
        loc.alive = False  # AMCL crashed; check_health has not run yet
    del loc.calls[:]
    svc.load_map('hall', Pose2D(1, 0, 0))
    assert loc.calls == [('stop',), ('localization', '/maps/hall/map.yaml', Pose2D(1, 0, 0))]
