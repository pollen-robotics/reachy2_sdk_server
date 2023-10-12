from collections import defaultdict, namedtuple
from typing import List

from .components import ComponentsHolder


Part = namedtuple("Part", ["name", "id", "type", "components"])


class PartsHolder:
    def __init__(self, logger, config: dict, components: ComponentsHolder) -> None:
        self.logger = logger

        self.logger.info("Creating parts.")

        config = config["reachy"]

        self.parts = {}
        self.by_name = {}
        self.by_id = {}
        self.by_type = defaultdict(list)

        part_id = 1

        for part in ("r_arm", "l_arm", "head"):
            if part in config:
                part_config = config[part]

                p = Part(
                    name=part,
                    id=part_id,
                    type=self.guess_part_type(part, part_config),
                    components=[
                        components.get_by_name(actuator["name"])
                        for actuator in part_config.values()
                    ],
                )

                self.parts[part] = p
                self.by_name[p.name] = p
                self.by_id[p.id] = p
                self.by_type[p.type].append(p)

                part_id += 1

                self.logger.info(f"\t - {p}")
        self.logger.info(f"Parts created (nb_parts={len(self.parts)}).\n")

    def get_by_type(self, part_type: str) -> List[Part]:
        return [p for p in self.parts.values() if p.type == part_type]

    def guess_part_type(self, part_name: str, part_config: dict) -> str:
        if part_name.endswith("_arm") and set(part_config.keys()) == {
            "shoulder",
            "elbow",
            "wrist",
        }:
            return "arm"
        else:
            raise ValueError(f"Unknown part type for {part_name}.")
