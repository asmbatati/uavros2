"""Where MAVROS puts its topics, which depends on the MAVROS release.

uavros2 expects MAVROS topics at ``/<ns>/mavros/<topic>`` (state,
local_position/pose, setpoint_position/local, cmd/arming, ...).

* mzahana's fork (2.6.0, what this workspace builds) names topics relative
  to the node's namespace, so the node runs in ``<ns>/mavros``.
* Upstream MAVROS as shipped for Jazzy (apt 2.16.0, what install.sh and the
  Docker image use) names them under the node's private namespace
  (``~/state``); with the node named ``mavros`` it must run in ``<ns>``,
  or every topic lands at ``/<ns>/mavros/mavros/<topic>``.

Verified on 2.6.0 and apt 2.16.0; releases in between are assumed to behave like
2.16. A build of mavlink/mavros HEAD (Nov 2026) also says 2.16.0 but publishes at
``/<ns>/state`` when run in ``<ns>``, so this check gets it wrong. Set
``UAVROS2_MAVROS_PRIVATE_TOPICS=0`` or ``=1`` to override.
"""

import os
import re
import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory


def _version(text):
    return tuple(int(x) for x in re.findall(r"\d+", text)[:3])


def mavros_version():
    """MAVROS version from its installed package.xml, e.g. (2, 16, 0)."""
    xml = os.path.join(get_package_share_directory("mavros"), "package.xml")
    return _version(ET.parse(xml).getroot().findtext("version", "0.0.0"))


def private_topics(version=None):
    """True if this MAVROS publishes under the node's private namespace."""
    override = os.environ.get("UAVROS2_MAVROS_PRIVATE_TOPICS")
    if override in ("0", "1"):
        return override == "1"
    if version is None:
        version = mavros_version()
    return version > (2, 6, 0)


def mavros_namespace(ns, version=None):
    """Namespace for the MAVROS node so its topics are /<ns>/mavros/<topic>."""
    ns = ns.strip("/")
    return ns if private_topics(version) else f"{ns}/mavros"
