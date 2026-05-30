"""Backend-agnostic VLM client (OpenAI-compatible /v1/chat/completions).

Local vLLM by default; any cloud endpoint with the same API works by changing
base_url/model/api_key. Uses only the standard library so the package adds no
pip dependency. The place_graph node re-validates everything this returns, so
this client is best-effort: on any failure it returns None and the caller
skips the region.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request

ROOM_TYPES = [
    'bedroom', 'living_room', 'kitchen', 'dining_room', 'bathroom', 'toilet',
    'office', 'study', 'meeting_room', 'hallway', 'corridor', 'foyer',
    'entryway', 'closet', 'storage', 'pantry', 'laundry_room', 'garage',
    'lobby', 'open_area', 'unknown',
]
AFFORDANCES = [
    'sleep', 'rest', 'cook', 'make_beverages', 'eat', 'wash', 'toilet',
    'work', 'meet', 'transit', 'wait', 'store', 'seating', 'entertainment',
    'plant_care',
]

RESULT_SCHEMA = {
    'type': 'object',
    'properties': {
        'semantic_label': {'type': 'string', 'enum': ROOM_TYPES},
        'instance_label': {'type': 'string'},
        'objects': {'type': 'array', 'items': {'type': 'string'}},
        'affordances': {
            'type': 'array',
            'items': {'type': 'string', 'enum': AFFORDANCES},
        },
        'summary': {'type': 'string'},
        'confidence': {'type': 'number'},
    },
    'required': ['semantic_label', 'confidence'],
}

SYSTEM_PROMPT = (
    'You label a region of an indoor building map from robot camera images. '
    'The images are different viewpoints of the SAME region and may be blurry, '
    'partial, or show a doorway into an adjacent room. Describe ONLY what is '
    'clearly part of this region, union the objects across views, and output '
    'ONE label for the whole region. Return JSON only.'
)


class VlmClient:
    def __init__(self, base_url, model, api_key='EMPTY', structured=True,
                 timeout=60.0, logger=None):
        self.base_url = base_url.rstrip('/')
        self.model = model
        self.api_key = api_key or 'EMPTY'
        self.structured = structured
        self.timeout = timeout
        self._log = logger

    def label_region(self, images_b64, geom_hint=''):
        """Return a dict with the structured label, or None on failure."""
        content = [{'type': 'text', 'text': self._user_text(geom_hint)}]
        for uri in images_b64:
            content.append({'type': 'image_url', 'image_url': {'url': uri}})
        payload = {
            'model': self.model,
            'messages': [
                {'role': 'system', 'content': SYSTEM_PROMPT},
                {'role': 'user', 'content': content},
            ],
            'temperature': 0.0,
            'max_tokens': 320,
            'response_format': {'type': 'json_object'},
        }
        if self.structured:
            # vLLM structured outputs (formerly guided_json). Harmless to local
            # vLLM; cloud APIs that reject unknown fields need vlm_structured
            # set false.
            payload['guided_json'] = RESULT_SCHEMA
        try:
            raw = self._post(payload)
        except (urllib.error.URLError, OSError, ValueError) as e:
            self._warn(f'VLM request failed: {e}')
            return None
        return self._parse(raw)

    def _user_text(self, geom_hint):
        vocab = (f"room_type must be one of: {', '.join(ROOM_TYPES)}. "
                 f"affordances must be from: {', '.join(AFFORDANCES)}.")
        instr = (
            'Identify the room type, the salient objects, the supported tasks '
            '(affordances), and a one-sentence summary. If the evidence is '
            'weak or mostly shows a doorway/adjacent room, use semantic_label '
            '"unknown" and confidence below 0.4. Output fields: semantic_label, '
            'instance_label, objects, affordances, summary, confidence (0-1).')
        return '\n'.join(s for s in (instr, vocab, geom_hint) if s)

    def _post(self, payload):
        url = f'{self.base_url}/chat/completions'
        body = json.dumps(payload).encode()
        req = urllib.request.Request(
            url, data=body, method='POST',
            headers={'Content-Type': 'application/json',
                     'Authorization': f'Bearer {self.api_key}'})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode())

    def _parse(self, raw):
        try:
            txt = raw['choices'][0]['message']['content']
            data = json.loads(txt)
        except (KeyError, IndexError, TypeError, json.JSONDecodeError) as e:
            self._warn(f'VLM parse failed: {e}')
            return None
        return data if isinstance(data, dict) else None

    def _warn(self, msg):
        if self._log is not None:
            self._log.warning(msg)
