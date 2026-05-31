"""VLM client for the qwenvl multipart inference gateway.

Talks to the gateway's ``POST /process`` endpoint, which takes
``multipart/form-data`` with a ``json_data`` JSON part and exactly one ``image``
file, and returns ``{"result": "<text>", "usage": {...}, ...}``. Stdlib-only so
the package adds no pip dependency beyond the numpy/PIL that ``frame_buffer``
already needs.

The gateway has no schema-enforced (guided/structured) decoding, so the model
returns free text that we extract JSON from best-effort. The place_graph node
re-validates everything this returns against its own vocab, so this client is
best-effort: on any failure it returns ``None`` and the caller skips the region.
"""
from __future__ import annotations

import json
import re
import urllib.error
import urllib.request
import uuid

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

# The gateway exposes a single `prompt` (no system/user split) and does no
# schema-enforced decoding, so the instruction has to carry the whole contract
# and insist on raw JSON.
PROMPT_HEADER = (
    'You label a region of an indoor building map from robot camera images. '
    'The image shows one or more viewpoints of the SAME region (when there are '
    'several, they are tiled into a single grid). Some tiles may be blurry, '
    'partial, or show a doorway into an adjacent room. Describe ONLY what is '
    'clearly part of this region, union the objects across the views, and '
    'output ONE label for the whole region.')


class VlmClient:
    def __init__(self, endpoint, robot_info='place_graph_labeler',
                 max_new_tokens=512, temperature=0.0, timeout=60.0,
                 logger=None):
        self.endpoint = self._normalize(endpoint)
        self.robot_info = robot_info or 'place_graph_labeler'
        # The gateway clamps these (tokens 1..4096, temp 0..2); clamp here too
        # so our values match what the server will actually use.
        self.max_new_tokens = max(1, min(4096, int(max_new_tokens)))
        self.temperature = max(0.0, min(2.0, float(temperature)))
        self.timeout = timeout
        self._log = logger

    @staticmethod
    def _normalize(endpoint):
        e = (endpoint or '').rstrip('/')
        if not e.endswith('/process'):
            e += '/process'
        return e

    def label_region(self, image_jpeg, geom_hint=''):
        """Label a region from one composited JPEG (raw bytes).

        Returns a dict with the structured label, or None on any failure.
        """
        if not image_jpeg:
            return None
        meta = {
            'service_type': 'qwenvl',
            'prompt': self._prompt(geom_hint),
            'robot_info': self.robot_info,
            'max_new_tokens': self.max_new_tokens,
            'temperature': self.temperature,
        }
        try:
            raw = self._post(meta, image_jpeg)
        except (urllib.error.URLError, OSError, ValueError) as e:
            self._warn(f'VLM request failed: {e}')
            return None
        return self._parse(raw)

    def _prompt(self, geom_hint):
        vocab = (f"semantic_label must be one of: {', '.join(ROOM_TYPES)}. "
                 f"affordances must be a subset of: {', '.join(AFFORDANCES)}.")
        instr = (
            'Identify the room type, the salient objects, the supported tasks '
            '(affordances), and a one-sentence summary. If the evidence is '
            'weak or mostly shows a doorway/adjacent room, use semantic_label '
            '"unknown" and confidence below 0.4.')
        fields = (
            'Return ONLY a JSON object (no prose, no markdown fences) with '
            'fields: semantic_label (string), instance_label (string), objects '
            '(array of strings), affordances (array of strings), summary '
            '(string), confidence (number 0-1).')
        return '\n'.join(s for s in
                         (PROMPT_HEADER, instr, vocab, fields, geom_hint) if s)

    def _post(self, meta, image_jpeg):
        boundary = '----placegraph' + uuid.uuid4().hex
        body = self._encode_multipart(boundary, meta, image_jpeg)
        req = urllib.request.Request(
            self.endpoint, data=body, method='POST',
            headers={'Content-Type':
                     f'multipart/form-data; boundary={boundary}',
                     'Content-Length': str(len(body))})
        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            return json.loads(resp.read().decode())

    @staticmethod
    def _encode_multipart(boundary, meta, image_jpeg, filename='region.jpg'):
        """Build a multipart/form-data body (binary-safe) by hand: a JSON
        `json_data` text part plus one `image` file part."""
        b = boundary.encode()
        crlf = b'\r\n'
        buf = bytearray()
        buf += b'--' + b + crlf
        buf += b'Content-Disposition: form-data; name="json_data"' + crlf
        buf += b'Content-Type: application/json' + crlf + crlf
        buf += json.dumps(meta).encode('utf-8') + crlf
        buf += b'--' + b + crlf
        buf += (f'Content-Disposition: form-data; name="image"; '
                f'filename="{filename}"').encode('utf-8') + crlf
        buf += b'Content-Type: image/jpeg' + crlf + crlf
        buf += image_jpeg + crlf
        buf += b'--' + b + b'--' + crlf
        return bytes(buf)

    def _parse(self, raw):
        try:
            text = raw['result']
        except (KeyError, TypeError) as e:
            self._warn(f'VLM response missing result: {e}')
            return None
        data = _extract_json(text)
        if not isinstance(data, dict):
            self._warn('VLM result was not a JSON object')
            return None
        return data

    def _warn(self, msg):
        if self._log is not None:
            self._log.warning(msg)


def _extract_json(text):
    """Best-effort: parse a JSON object out of free-form model text, tolerating
    ```json fences or leading/trailing prose."""
    if not isinstance(text, str):
        return None
    s = text.strip()
    if s.startswith('```'):
        s = re.sub(r'^```[a-zA-Z]*\s*', '', s)
        s = re.sub(r'\s*```$', '', s).strip()
    try:
        return json.loads(s)
    except json.JSONDecodeError:
        pass
    start, end = s.find('{'), s.rfind('}')
    if 0 <= start < end:
        try:
            return json.loads(s[start:end + 1])
        except json.JSONDecodeError:
            return None
    return None
