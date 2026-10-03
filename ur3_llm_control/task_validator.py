#!/usr/bin/env python3

ALLOWED_SKILLS = {"pick", "place", "home"}
ALLOWED_OBJECTS = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"}
ALLOWED_ZONES = {"zone_a", "zone_b", "zone_c"}

REQUIRED_FIELDS = {
    "pick": {"object"},
    "place": {"object", "zone"},
    "home": set(),
}


def validate_plan(plan: dict) -> tuple[bool, str]:
    if "plan" not in plan or not isinstance(plan["plan"], list):
        return False, "INVALID_FORMAT: missing 'plan' list"

    if len(plan["plan"]) == 0:
        return False, "INVALID_FORMAT: empty plan"

    
    zone_holder = {}

    for i, step in enumerate(plan["plan"]):
        skill = step.get("skill")

        if skill not in ALLOWED_SKILLS:
            return False, f"INVALID_SKILL at step {i}: {skill}"

        required = REQUIRED_FIELDS[skill]
        missing = required - step.keys()
        if missing:
            return False, f"MISSING_FIELD at step {i} ({skill}): {missing}"

        if "object" in step and step["object"] not in ALLOWED_OBJECTS:
            return False, f"INVALID_OBJECT at step {i}: {step['object']}"

        if "zone" in step and step["zone"] not in ALLOWED_ZONES:
            return False, f"INVALID_ZONE at step {i}: {step['zone']}"

        if skill == "place":
            zone = step["zone"]
            obj = step["object"]
            occupant = zone_holder.get(zone)
            if occupant is not None and occupant != obj:
                return False, (
                    f"ZONE_CONFLICT at step {i}: zone '{zone}' dang chua '{occupant}', "
                    f"khong the dat '{obj}' vao truoc khi doi '{occupant}' di noi khac "
                    f"(can chen buoc trung gian qua diem do)"
                )
            zone_holder[zone] = obj

    return True, "OK"
