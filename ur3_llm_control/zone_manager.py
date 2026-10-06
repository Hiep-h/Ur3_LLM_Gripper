#!/usr/bin/env python3
ZONE_RADIUS = 0.06
# Hai ngon gripper mo theo truc x (+-7cm), be rong 2cm theo y: khoi o canh bi cham neu |dx|<0.12 va |dy|<0.05.
NEIGHBOR_DX = 0.12
NEIGHBOR_DY = 0.05


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
        self.object_xy = {}  # vi tri uoc luong cua tung khoi (camera, roi cap nhat theo ke hoach)

    def sync_from_camera(self, detected: dict):
        """detected: {object_name: (x, y)} tu perception.detect_objects()."""
        self.occupancy = {name: None for name in self.positions}
        self.object_xy = {obj: (x, y) for obj, (x, y) in detected.items()}
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
        self.object_xy[obj] = (self.positions[zone][0], self.positions[zone][1])

    def location_of(self, obj: str):
        for z, o in self.occupancy.items():
            if o == obj:
                return z
        return None

    def clear(self, zone: str):
        self.occupancy[zone] = None

    def _near_other(self, name: str, exclude_obj: str | None = None) -> bool:
        """Co khoi nao dang nam du gan diem `name` de ngon gripper cham vao (khi gap/tha o do)?"""
        px, py = self.positions[name][0], self.positions[name][1]
        for obj, (x, y) in self.object_xy.items():
            if obj == exclude_obj:
                continue
            if abs(x - px) < NEIGHBOR_DX and abs(y - py) < NEIGHBOR_DY and \
                    ((x - px) ** 2 + (y - py) ** 2) ** 0.5 > 0.03:
                return True
        return False

    def free_park(self, exclude: set | None = None, obj: str | None = None) -> str | None:
        """Diem do tam TRONG va khong sat khoi khac (uu tien); neu khong co thi chap nhan diem trong bat ky."""
        exclude = exclude or set()
        free = [n for n in self.park_names if n not in exclude and self.occupancy[n] is None]
        for name in free:
            if not self._near_other(name, obj):
                return name
        return free[0] if free else None


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
            current = zm.location_of(obj)
            if target_zone == "temp":
                if current in zm.park_names:
                    i += 2  # da nam o diem do tam: bo qua
                    continue
                park = zm.free_park(obj=obj)
                if park is None:
                    raise RuntimeError(f"Khong con diem do tam trong cho '{obj}'")
                place_step = {**place_step, "zone": park}
                target_zone = park
            elif current == target_zone:
                i += 2  # da dung zone: bo qua
                continue
            occupant = zm.object_at(target_zone)

            if occupant is not None and occupant != obj:
                park = zm.free_park(obj=occupant)
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
