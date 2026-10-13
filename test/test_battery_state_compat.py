import math

import pytest
from control_msgs.msg import BatteryStateArray
from sensor_msgs.msg import BatteryState

from articubot_one.battery_state_compat import normalize_battery_state, select_battery


@pytest.mark.parametrize('percentage,expected', [
    (0.0, 0.0), (50.0, 0.5), (100.0, 1.0), (-1.0, 0.0), (101.0, 1.0),
])
def test_percentage_scale(percentage, expected):
    original = BatteryState(percentage=percentage)
    result = normalize_battery_state(original, 'battery_frame')
    assert result.percentage == pytest.approx(expected)
    assert original.percentage == percentage


@pytest.mark.parametrize('percentage', [math.nan, math.inf, -math.inf])
def test_unknown_percentage_stays_unknown(percentage):
    result = normalize_battery_state(BatteryState(percentage=percentage), 'battery_frame')
    assert math.isnan(result.percentage)


def test_raw_presence_metadata_and_measurements_are_preserved():
    original = BatteryState(
        voltage=12.5, current=-1.0, temperature=21.0, charge=9.0,
        capacity=18.0, design_capacity=20.0, percentage=50.0,
        present=False, location='robot Plucky', serial_number='3.0001',
        power_supply_status=BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
        power_supply_health=BatteryState.POWER_SUPPLY_HEALTH_GOOD,
        power_supply_technology=BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE,
    )
    original.header.frame_id = 'main_battery'
    original.header.stamp.sec = 123
    result = select_battery(BatteryStateArray(battery_states=[original]),
                            'main_battery', 'battery_frame')
    assert result is not None
    assert result.present is False
    assert result.header.frame_id == 'battery_frame'
    assert original.header.frame_id == 'main_battery'
    result.header.frame_id = original.header.frame_id
    result.percentage = original.percentage
    assert result == original


def test_missing_or_duplicate_battery_is_not_published():
    original = BatteryState()
    original.header.frame_id = 'other_battery'
    assert select_battery(BatteryStateArray(battery_states=[original]),
                          'main_battery', 'battery_frame') is None
    original.header.frame_id = 'main_battery'
    assert select_battery(BatteryStateArray(battery_states=[original, original]),
                          'main_battery', 'battery_frame') is None
