#!/usr/bin/env python3
ZONE_RADIUS = 0.06


class ZoneManager:
    """Theo doi zone/diem do nao dang chua vat nao.

    Trang thai khoi tao tu camera (sync_from_camera) va duoc cap nhat qua
    tung buoc cua ke hoach (set_object), khong dung snapshot tinh.
    """

    def __init__(self, zones: dict, parks: dict):
        self.positions = {**zones, **parks}
        self.zone_names = list(zones)
        self.park_names = list(parks)
        self.occupancy = {name: None for name in self.positions}

    def sync_from_camera(self, detected: dict):
        """detected: {object_name: (x, y)} tu perception.detect_objects()."""
        self.occupancy = {name: None for name in self.positions}
        for obj, (x, y) in detected.items():
            for name, pos in self.positions.items():
                if ((x - pos[0]) ** 2 + (y - pos[1]) ** 2) ** 0.5 <= ZONE_RADIUS:
                    self.occupancy[name] = obj
                    break

    def object_at(self, zone: str):
        return self.occupancy.get(zone)

    def set_object(self, zone: str, obj: str):
        for z, o in self.occupancy.items():
            if o == obj:
                self.occupancy[z] = None
        self.occupancy[zone] = obj

    def clear(self, zone: str):
        self.occupancy[zone] = None

    def free_park(self, exclude: set | None = None) -> str | None:
        exclude = exclude or set()
        for name in self.park_names:
            if name not in exclude and self.occupancy[name] is None:
                return name
        return None


def expand_plan_with_conflict_resolution(steps: list[dict], zm: ZoneManager) -> list[dict]:
    """Chen buoc don vat can sang diem do khi zone dich dang bi chiem."""
    new_steps = []
    i = 0
    while i < len(steps):
        step = steps[i]
        if (step["skill"] == "pick" and i + 1 < len(steps)
                and steps[i + 1]["skill"] == "place"):
            obj = step["object"]
            place_step = steps[i + 1]
            target_zone = place_step["zone"]
            occupant = zm.object_at(target_zone)

            if occupant is not None and occupant != obj:
                park = zm.free_park()
                if park is None:
                    raise RuntimeError(
                        f"Khong con diem do trong de don vat '{occupant}' khoi {target_zone}"
                    )
                new_steps.append({"skill": "pick", "object": occupant})
                new_steps.append({"skill": "place", "object": occupant, "zone": park})
                zm.set_object(park, occupant)

            new_steps.append(step)
            new_steps.append(place_step)
            zm.set_object(target_zone, obj)
            i += 2
        else:
            new_steps.append(step)
            if step["skill"] == "place":
                zm.set_object(step["zone"], step["object"])
            i += 1
    return new_steps
