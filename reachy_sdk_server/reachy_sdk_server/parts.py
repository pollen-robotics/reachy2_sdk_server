from collections import defaultdict, namedtuple
from typing import List, Optional

from reachy2_sdk_api.part_pb2 import PartId

from .components import ComponentsHolder

Part = namedtuple("Part", ["name", "id", "type", "components", "components_dict"])


class PartsHolder:
    def __init__(self, logger, config: dict, components: ComponentsHolder) -> None:
        self.logger = logger

        self.logger.info("Creating parts.")

        mobile_base_config = config["mobile_base"]
        config = config["reachy"]

        self.parts = {}
        self.by_name = {}
        self.by_id = {}
        self.by_type = defaultdict(list)

        # We start at 1 to avoid having a part_id of 0
        part_id = 1

        for part in ("r_arm", "l_arm", "head", "l_hand", "r_hand"):
            if part in config:
                part_config = config[part]
                prefix = "" if not "_" in part else part.split("_")[0] + "_"
                p = Part(
                    name=part,
                    id=part_id,
                    type=self.guess_part_type(part, part_config),
                    components=[components.get_by_name(actuator["name"]) for actuator in part_config.values()],
                    # if part has prefix, remove prefix from dict key, for generic parsing later
                    components_dict={
                        actuator["name"].replace(prefix, ""): components.get_by_name(actuator["name"])
                        for actuator in part_config.values()
                    },
                )

                self.parts[part] = p
                self.by_name[p.name] = p
                self.by_id[p.id] = p
                self.by_type[p.type].append(p)

                part_id += 1

                self.logger.info(f"\t - {p}")

        if mobile_base_config["serial_number"] != "null":
            part = "mobile_base"
            p = Part(
                name=part,
                id=part_id,
                type=self.guess_part_type(part, dict()),
                components=[],  # No components for the mobile base
                components_dict=dict(),
            )

            self.parts[part] = p
            self.by_name[p.name] = p
            self.by_id[p.id] = p
            self.by_type[p.type].append(p)

            self.logger.info(f"\t - {p}")

        self.logger.info(f"Parts created (nb_parts={len(self.parts)}).\n")

    def __iter__(self):
        return iter(self.parts.values())

    def get_by_part_id(self, part_id: PartId) -> Optional[Part]:
        if part_id.id:
            return self.get_by_id(part_id.id)
        elif part_id.name:
            return self.get_by_name(part_id.name)
        return None

    def get_by_name(self, part_name: str) -> Optional[Part]:
        if part_name in self.by_name:
            return self.by_name[part_name]

    def get_by_id(self, part_id: int) -> Optional[Part]:
        if part_id in self.by_id:
            return self.by_id[part_id]

    def get_by_type(self, part_type: str) -> List[Part]:
        return [p for p in self.parts.values() if p.type == part_type]

    def guess_part_type(self, part_name: str, part_config: dict) -> str:
        if part_name.endswith("_arm") and set(part_config.keys()) == {
            "shoulder",
            "elbow",
            "wrist",
        }:
            return "arm"
        elif part_name == "head" and set(part_config.keys()) == {
            "neck",
            "l_antenna",
            "r_antenna",
        }:
            return "head"
        elif part_name.endswith("_hand"):
            return "hand"
        elif part_name == "mobile_base":
            return "mobile_base"
        else:
            raise ValueError(f"Unknown part type for {part_name}.")
