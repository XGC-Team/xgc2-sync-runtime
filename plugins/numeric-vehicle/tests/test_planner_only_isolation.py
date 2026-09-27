"""Static regression for the EXISTING planner-only graph, not full-HIL evidence.

Uses only a checked-in TOML template; never starts ROS, transport or hardware.
Run from the repository root with Python 3.11+:
    python -m unittest discover -s plugins/numeric-vehicle/tests -p 'test_*.py' -v
"""
from copy import deepcopy
from pathlib import Path
import tomllib
import unittest

ROOT = Path(__file__).resolve().parents[3]
COMPOSITION = ROOT / "crates/xgc-rt-host/src/deployment/composition-numeric-hil.toml"
ROLES = {"ros_io", "station_io", "numeric_vehicle", "dmpc_rounds", "plan_dmpc"}
ROS_PORTS = {"scene_snapshot", "scene_heartbeat", "timeline_ack", "timeline_status"}
PLANT_PORTS = {"command", "position_target", "paired_state", "controller_state"}
PHYSICAL_PORTS = {"setpoint", "fcu_request", "attitude_rate", "actuator", "actuator_control"}


def check_planner_only_isolation(graph: dict) -> None:
    """Check this release-role template only, not arbitrary rendered manifests."""
    plugins = graph["plugin"]
    roles = [plugin["path"] for plugin in plugins]
    names = [plugin["name"] for plugin in plugins]
    if len(roles) != len(set(roles)) or len(names) != len(set(names)):
        raise ValueError("duplicate plugin identity")
    if set(roles) != ROLES:
        raise ValueError("not the existing planner-only role set")
    for plugin in plugins:
        ports = set(plugin["bind"])
        if ports & PHYSICAL_PORTS:
            raise ValueError("physical output/service binding in planner-only graph")
        if plugin["path"] == "ros_io" and ports != ROS_PORTS:
            raise ValueError("ROS edge must stay scene/status only")
        if plugin["path"] == "numeric_vehicle" and ports != PLANT_PORTS:
            raise ValueError("planner-only model ports changed")


class PlannerOnlyIsolationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with COMPOSITION.open("rb") as source:
            cls.graph = tomllib.load(source)

    def plugin(self, graph, role):
        return next(plugin for plugin in graph["plugin"] if plugin["path"] == role)

    def test_existing_template_is_planner_only(self):
        check_planner_only_isolation(self.graph)

    def test_no_controller_is_present_not_a_full_hil_claim(self):
        roles = {plugin["path"] for plugin in self.graph["plugin"]}
        self.assertEqual(roles, ROLES)
        self.assertNotIn("controller", roles)

    def test_baseline_planner_pva_really_feeds_ideal_model(self):
        planner = self.plugin(self.graph, "plan_dmpc")["bind"]["position_target"]
        plant = self.plugin(self.graph, "numeric_vehicle")["bind"]["position_target"]
        self.assertEqual(planner["channel"], plant["channel"])
        self.assertEqual(plant["from"], ["SELF"])

    def test_state_feedback_is_local_model_not_injected_paired_state(self):
        plant = self.plugin(self.graph, "numeric_vehicle")["bind"]
        planner = self.plugin(self.graph, "plan_dmpc")["bind"]
        for port in ("paired_state", "controller_state"):
            self.assertEqual(plant[port], {"channel": port})
            self.assertEqual(planner[port], {"channel": port, "from": ["SELF"]})

    def test_commands_are_authority_scoped_model_inputs(self):
        plant = self.plugin(self.graph, "numeric_vehicle")["bind"]
        self.assertEqual(plant["command"], {"channel": "command", "from": ["AUTHORITY"]})

    def test_original_period_wake_and_planner_budget_are_unchanged(self):
        self.assertEqual(self.graph["session"]["period_ms"], 1)
        plant = self.plugin(self.graph, "numeric_vehicle")
        self.assertEqual((plant["trigger"], plant["wake_ms"]), ("on_dirty", 10))
        self.assertEqual(self.plugin(self.graph, "plan_dmpc")["step_budget_ms"], 80)

    def test_rejects_hardware_ports_even_on_innocently_named_channels(self):
        for port in PHYSICAL_PORTS:
            with self.subTest(port=port):
                graph = deepcopy(self.graph)
                self.plugin(graph, "ros_io")["bind"][port] = {"channel": "harmless_name"}
                with self.assertRaises(ValueError):
                    check_planner_only_isolation(graph)

    def test_rejects_unknown_ros_edge_ports(self):
        graph = deepcopy(self.graph)
        self.plugin(graph, "ros_io")["bind"]["new_hardware_port"] = {"channel": "other"}
        with self.assertRaises(ValueError):
            check_planner_only_isolation(graph)

    def test_rejects_a_real_controller_added_to_the_old_graph(self):
        graph = deepcopy(self.graph)
        graph["plugin"].append({"name": "ctl-px4", "path": "controller", "bind": {}})
        with self.assertRaises(ValueError):
            check_planner_only_isolation(graph)

    def test_rejects_unknown_or_missing_role(self):
        for replacement in (None, "unreviewed_plugin"):
            with self.subTest(replacement=replacement):
                graph = deepcopy(self.graph)
                if replacement is None:
                    graph["plugin"].pop()
                else:
                    graph["plugin"][0]["path"] = replacement
                with self.assertRaises(ValueError):
                    check_planner_only_isolation(graph)

    def test_rejects_duplicate_plugin_identity(self):
        for field in ("name", "path"):
            with self.subTest(field=field):
                graph = deepcopy(self.graph)
                graph["plugin"][1][field] = graph["plugin"][0][field]
                with self.assertRaises(ValueError):
                    check_planner_only_isolation(graph)

    def test_rejects_control_input_silently_added_to_planner_only_plant(self):
        graph = deepcopy(self.graph)
        self.plugin(graph, "numeric_vehicle")["bind"]["controller_output"] = {"channel": "control"}
        with self.assertRaises(ValueError):
            check_planner_only_isolation(graph)


if __name__ == "__main__":
    unittest.main()
