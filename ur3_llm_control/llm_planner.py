#!/usr/bin/env python3
import json
import re
from openai import OpenAI
BASE_PROMPT_TEMPLATE = """Available skills:
pick(object)
place(object, zone)
home()

Objects: red_cube, yellow_cube, blue_cube, green_cube, purple_cube
Zones: zone_a, zone_b, zone_c, temp
  (temp = a temporary parking spot; the execution system chooses a free one. Use it for objects that have no zone.)

Vietnamese vocabulary mapping (user may write in Vietnamese or English, in many phrasings):
  "khoi do" / "mau do" / "red" -> red_cube
  "khoi vang" / "mau vang" / "yellow" -> yellow_cube
  "khoi xanh" / "mau xanh duong" / "blue" -> blue_cube
  "khoi xanh la" / "mau xanh la" / "green" -> green_cube
  "khoi tim" / "mau tim" / "purple" -> purple_cube
  "vung A" / "zone A" / "khu A" / "ô A" -> zone_a
  "vung B" / "zone B" / "khu B" / "ô B" -> zone_b
  "vung C" / "zone C" / "khu C" / "ô C" -> zone_c

You do NOT need to worry about a target zone being occupied by another object — that is handled automatically by the execution system after your plan is returned. Just generate the direct pick/place plan for what the user asked.

Example:
User: "Đưa khối màu đỏ vào vùng B."
Plan: {{"plan": [{{"skill": "pick", "object": "red_cube"}}, {{"skill": "place", "object": "red_cube", "zone": "zone_b"}}, {{"skill": "home"}}]}}

{personalization_block}

Return ONLY a JSON execution plan, no explanation, no markdown fences.
Format:
{{"plan": [{{"skill": "pick", "object": "..."}}, {{"skill": "place", "object": "...", "zone": "..."}}, {{"skill": "home"}}]}}
"""
class LLMPlanner:
    def __init__(self, base_url: str, api_key: str, model: str,
                 zone_mapping: dict | None = None):
        self.client = OpenAI(base_url=base_url, api_key=api_key)
        self.model = model
        self.zone_mapping = zone_mapping or {}

    def _build_system_prompt(self) -> str:
        if self.zone_mapping:
            mapping_lines = "\n".join(
                f"  {zone} -> {obj}" for zone, obj in self.zone_mapping.items()
            )
            personalization_block = (
                "SORTING RULE: if the user asks to sort / arrange / organize objects by colour, to their "
                "colour zone, or 'according to my student ID' (Vietnamese: 'sap xep', 'theo mau', "
                "'dung zone', 'theo MSSV'), use exactly this fixed mapping (do not invent another one):\n"
                f"{mapping_lines}\n"
                "Plan ONE pick + place pair for EVERY object in the Objects list, in this order: first the "
                "objects that appear in the mapping (place each in its zone), then every remaining object "
                "(not in the mapping): pick it and place it in zone 'temp'. End with home. "
                "Example for this mapping: pick/place each mapped object to its zone, then pick/place each "
                "other object with zone temp, then home."
            )
        else:
            personalization_block = ""
        return BASE_PROMPT_TEMPLATE.format(personalization_block=personalization_block)

    def _extract_json(self, raw: str) -> str:
        raw = raw.replace("```json", "").replace("```", "").strip()
        match = re.search(r"\{.*\}", raw, re.DOTALL)
        return match.group(0) if match else raw

    def get_plan(self, user_command: str, retry: bool = True) -> dict:
        system_prompt = self._build_system_prompt()
        
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command},
                ],
                response_format={"type": "json_object"},
                temperature=0.0
            )
        except Exception:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command},
                ],
                temperature=0.0
            )

        raw = response.choices[0].message.content.strip()

        try:
            return json.loads(self._extract_json(raw))
        except json.JSONDecodeError:
            if not retry:
                raise
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_command},
                    {"role": "user", "content": "Reply with ONLY valid JSON, nothing else."},
                ],
                temperature=0.0
            )
            raw2 = response.choices[0].message.content.strip()
            return json.loads(self._extract_json(raw2))
