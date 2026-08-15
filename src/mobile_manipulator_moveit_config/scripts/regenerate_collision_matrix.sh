#!/usr/bin/env bash
# regenerate_collision_matrix.sh
# ─────────────────────────────────────────────────────────────────────────────
# Phase 5 — regenerate config/mobile_manipulator.srdf from
# config/mobile_manipulator.srdf.base using the MoveIt Setup Assistant's OWN
# self-collision sampler (collisions_updater, the headless entry point of the
# assistant's "Self-Collisions" step) at its default settings:
#
#   --trials 10000                 assistant's default sampling density
#   --min-collision-fraction 0.95  assistant's default "always colliding" cutoff
#   --default --always             disable default-state and always-colliding pairs
#
# Run it after any change to the URDF or to the planning groups:
#
#   src/mobile_manipulator_moveit_config/scripts/regenerate_collision_matrix.sh
#
# Everything except <disable_collisions> lives in the .srdf.base file, so the
# hand-authored semantics are never clobbered by a regeneration.
#
# WITH ONE EXCEPTION, re-merged by this script (Phase 9).  collisions_updater
# does NOT carry the input SRDF's own <disable_collisions> entries through to
# its output — it writes only the pairs its sampler derived, silently dropping
# anything hand-authored.  The sampler's three-way classification (Default /
# Always / Never) also has no category for "in collision over part of one
# joint's range", which is precisely the camera_link vs left_knuckle case
# documented in the .base file.  So after the sampler runs, this script splices
# any <disable_collisions> found in the .base back into the generated file.
# ─────────────────────────────────────────────────────────────────────────────
set -eo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# source before enabling -u: setup.bash reads AMENT_TRACE_SETUP_FILES unguarded
source /opt/ros/humble/setup.bash
# overlay: collisions_updater resolves this package + the description
# package through ament_index, so the workspace must be sourced too
WS_SETUP="${PKG_DIR}/../../install/setup.bash"
[ -f "${WS_SETUP}" ] && source "${WS_SETUP}"
set -u

/opt/ros/humble/lib/moveit_setup_assistant/collisions_updater \
  --urdf   "${PKG_DIR}/config/mobile_manipulator.urdf.xacro" \
  --srdf   "${PKG_DIR}/config/mobile_manipulator.srdf.base" \
  --output "${PKG_DIR}/config/mobile_manipulator.srdf" \
  --xacro-args "sensor_arch:=0" \
  --default --always --verbose \
  --trials 10000 \
  --min-collision-fraction 0.95

# ── re-merge the hand-authored exceptions the sampler drops ──────────────────
# Parsed as XML, never regex-scraped: the .base file discusses
# <disable_collisions> inside its own comments, and a regex that does not know
# the difference happily matches the mention instead of the tag and splices a
# second <robot> element into the output.
python3 - "${PKG_DIR}/config/mobile_manipulator.srdf.base" \
         "${PKG_DIR}/config/mobile_manipulator.srdf" <<'PY'
import sys
import xml.etree.ElementTree as ET

base_path, out_path = sys.argv[1], sys.argv[2]
wanted = [(e.get('link1'), e.get('link2'), e.get('reason') or 'hand-authored')
          for e in ET.parse(base_path).getroot().findall('disable_collisions')]

out_root = ET.parse(out_path).getroot()
have = {frozenset((e.get('link1'), e.get('link2')))
        for e in out_root.findall('disable_collisions')}

kept = [(a, b, r) for a, b, r in wanted if frozenset((a, b)) not in have]
if kept:
    text = open(out_path).read().rstrip()
    assert text.endswith('</robot>'), 'unexpected generated SRDF layout'
    lines = ['', '    <!--Hand-authored in mobile_manipulator.srdf.base and re-merged by',
             '        regenerate_collision_matrix.sh: the sampler drops the input SRDF\'s',
             '        own entries, and its Default/Always/Never classification cannot',
             '        express a pair that collides over only part of a joint\'s range.',
             '        The rationale for each entry lives in the .base file.-->']
    for a, b, r in kept:
        lines.append('    <disable_collisions link1="%s" link2="%s" reason="%s"/>'
                     % (a, b, r))
    merged = (text[:-len('</robot>')].rstrip() + '\n'
              + '\n'.join(lines) + '\n</robot>\n')
    open(out_path, 'w').write(merged)
print('re-merged %d hand-authored exception(s) from the .base' % len(kept))
PY

echo "Wrote ${PKG_DIR}/config/mobile_manipulator.srdf"
grep -c disable_collisions "${PKG_DIR}/config/mobile_manipulator.srdf" \
  | xargs echo "disabled pairs:"
