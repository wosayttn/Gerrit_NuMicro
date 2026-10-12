# Eclipse Paho MQTT Embedded C Third-Party Component Description (for SCA / SBOM)

This document provides SBOM-ready metadata for the vendored Eclipse Paho MQTT Embedded C component under `ThirdParty/paho.mqtt.embedded-c`.

## 1) Component Identity

- Component name (`name`): `Eclipse Paho MQTT Embedded C`
- Component type (`type`): `library`
- Supplier / project: `Ian Craggs / Eclipse Paho project`
- Version: `v1.1.0-6035ea2` (tag + commit)
- Release tag context: latest observed upstream tag is `v1.1.0` (2017), while this BSP uses a later `master` commit.
- License: `Eclipse Public License v2.0 OR Eclipse Distribution License v1.0`
- Evidence path: `ThirdParty/paho.mqtt.embedded-c`
- Included top-level paths: `.settings`, `Debug`, `doc`, `MQTTClient`, `MQTTClient-C`, `MQTTPacket`, `test`, `CMakeLists.txt`, `library.properties`, `LICENSE`, `NOTICE`, `README.md`, `about.html`, `notice.html`, `edl-v10`, `epl-v20`

## 2) Evidence for Version and License

Primary evidence in this repository:

- `ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt`
  - Defines `PAHO_VERSION_MAJOR` as `1`.
  - Defines `PAHO_VERSION_MINOR` as `0`.
  - Defines `PAHO_VERSION_PATCH` as `0`.
  - Constructs `CLIENT_VERSION` from those version macros.
  - Header states `Eclipse Public License v2.0` and `Eclipse Distribution License v1.0`.
- Upstream repository history reference
  - Latest release tag observed: `v1.1.0` (2017).
  - Current integrated source in BSP is pinned to commit `6035ea2` on `master`.
- `ThirdParty/paho.mqtt.embedded-c/library.properties`
  - Defines `version=1.0.0`.
  - Identifies the Arduino library name as `MQTTClient`.
  - Identifies the project URL as `https://github.com/eclipse/paho.mqtt.embedded-c`.
- `ThirdParty/paho.mqtt.embedded-c/LICENSE`
  - Declares `EPL-2.0` and `EDL v1.0` as distribution licenses.
- `ThirdParty/paho.mqtt.embedded-c/NOTICE`
  - Declares project licenses as `EPL-2.0` or `EDL v1.0`.
  - Contains `SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause`.
- `ThirdParty/paho.mqtt.embedded-c/about.html`
  - States `Eclipse Public License Version 2.0` and `Eclipse Distribution License Version 1.0`.
- `ThirdParty/paho.mqtt.embedded-c/README.md`
  - Identifies this repository as Eclipse Paho MQTT Embedded C/C++ library.
  - States dual licensing under EPL and EDL.
- `ThirdParty/paho.mqtt.embedded-c/epl-v20` and `ThirdParty/paho.mqtt.embedded-c/edl-v10`
  - Provide bundled license texts for EPL 2.0 and EDL 1.0.

## 3) License Handling Guidance

For CycloneDX output, preserve upstream dual-license wording:

- `licenses[0].license.name = Eclipse Public License 2.0 OR Eclipse Distribution License 1.0`

Also preserve the upstream `LICENSE`, `NOTICE`, `epl-v20`, `edl-v10`, `about.html`, `notice.html`, and source-file copyright/license notices.

## 4) Suggested CycloneDX Field Mapping

Recommended component fields:

- `type`: `library`
- `name`: `Eclipse Paho MQTT Embedded C`
- `version`: `v1.1.0-6035ea2`
- `scope`: `required`
- `author`: `Ian Craggs / Eclipse Paho project`
- `description`: `Eclipse Paho MQTT C/C++ client library for embedded platforms vendored in ThirdParty/paho.mqtt.embedded-c, pinned to upstream commit 6035ea2 on master (latest release tag observed: v1.1.0).`
- `licenses[0].license.name`: `Eclipse Public License 2.0 OR Eclipse Distribution License 1.0`
- `properties` (recommended custom properties):
  - `src_path = ThirdParty/paho.mqtt.embedded-c`
  - `integration = vendored_source`
  - `paho_mqtt_embedded_c_version = v1.1.0-6035ea2`
    - `paho_mqtt_embedded_c_latest_release_tag = v1.1.0`
    - `paho_mqtt_embedded_c_version_scheme = commit_hash`
  - `paho_mqtt_embedded_c_included_top_level_paths = .settings; Debug; doc; MQTTClient; MQTTClient-C; MQTTPacket; test; CMakeLists.txt; library.properties; LICENSE; NOTICE; README.md; about.html; notice.html; edl-v10; epl-v20`
  - `bsp:component-origin = third-party`
  - `bsp:component-source = Eclipse Paho MQTT Embedded C`
  - `bsp:evidence-file = Document/SBOM/components/TestSampleView/sca_mqtt.json`
  - `bsp:evidence-path = ThirdParty/paho.mqtt.embedded-c`
  - `bsp:version-evidence = Pinned upstream commit: 6035ea2 (master); latest release tag: v1.1.0; ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt; ThirdParty/paho.mqtt.embedded-c/library.properties`
  - `bsp:license-evidence = ThirdParty/paho.mqtt.embedded-c/LICENSE; ThirdParty/paho.mqtt.embedded-c/NOTICE; ThirdParty/paho.mqtt.embedded-c/README.md; ThirdParty/paho.mqtt.embedded-c/about.html; ThirdParty/paho.mqtt.embedded-c/notice.html; ThirdParty/paho.mqtt.embedded-c/epl-v20; ThirdParty/paho.mqtt.embedded-c/edl-v10; ThirdParty/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.h`

## 5) Suggested BOM-Ref and purl

Suggested values:

- `bom-ref`: `pkg:generic/paho.mqtt.embedded-c@v1.1.0-6035ea2?source=vendored&path=ThirdParty/paho.mqtt.embedded-c`
- `purl`: `pkg:generic/paho.mqtt.embedded-c@v1.1.0-6035ea2`

If your internal SBOM naming policy differs, keep naming consistent across all third-party components in this BSP.

## 6) CycloneDX JSON Component Example

```json
{
  "type": "library",
  "bom-ref": "pkg:generic/paho.mqtt.embedded-c@v1.1.0-6035ea2?source=vendored&path=ThirdParty/paho.mqtt.embedded-c",
  "name": "Eclipse Paho MQTT Embedded C",
  "version": "v1.1.0-6035ea2",
  "scope": "required",
  "author": "Ian Craggs / Eclipse Paho project",
  "purl": "pkg:generic/paho.mqtt.embedded-c@v1.1.0-6035ea2",
  "description": "Eclipse Paho MQTT C/C++ client library for embedded platforms vendored in ThirdParty/paho.mqtt.embedded-c, pinned to upstream commit 6035ea2 on master (latest release tag observed: v1.1.0).",
  "licenses": [
    {
      "license": {
        "name": "Eclipse Public License 2.0 OR Eclipse Distribution License 1.0"
      }
    }
  ],
  "properties": [
    { "name": "src_path", "value": "ThirdParty/paho.mqtt.embedded-c" },
    { "name": "integration", "value": "vendored_source" },
    { "name": "paho_mqtt_embedded_c_version", "value": "v1.1.0-6035ea2" },
    { "name": "paho_mqtt_embedded_c_latest_release_tag", "value": "v1.1.0" },
    { "name": "paho_mqtt_embedded_c_version_scheme", "value": "commit_hash" },
    { "name": "paho_mqtt_embedded_c_included_top_level_paths", "value": ".settings; Debug; doc; MQTTClient; MQTTClient-C; MQTTPacket; test; CMakeLists.txt; library.properties; LICENSE; NOTICE; README.md; about.html; notice.html; edl-v10; epl-v20" },
    { "name": "bsp:component-origin", "value": "third-party" },
    { "name": "bsp:component-source", "value": "Eclipse Paho MQTT Embedded C" },
    { "name": "bsp:evidence-file", "value": "Document/SBOM/components/TestSampleView/sca_mqtt.json" },
    { "name": "bsp:evidence-path", "value": "ThirdParty/paho.mqtt.embedded-c" },
    { "name": "bsp:version-evidence", "value": "Pinned upstream commit: 6035ea2 (master); latest release tag: v1.1.0; ThirdParty/paho.mqtt.embedded-c/CMakeLists.txt; ThirdParty/paho.mqtt.embedded-c/library.properties" },
    { "name": "bsp:license-evidence", "value": "ThirdParty/paho.mqtt.embedded-c/LICENSE; ThirdParty/paho.mqtt.embedded-c/NOTICE; ThirdParty/paho.mqtt.embedded-c/README.md; ThirdParty/paho.mqtt.embedded-c/about.html; ThirdParty/paho.mqtt.embedded-c/notice.html; ThirdParty/paho.mqtt.embedded-c/epl-v20; ThirdParty/paho.mqtt.embedded-c/edl-v10; ThirdParty/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.h" }
  ]
}
```

## 7) Compliance Notes

- Keep original upstream copyright/license notices.
- Version evidence is taken from `CMakeLists.txt` and `library.properties`.
- License evidence is taken from `LICENSE`, `NOTICE`, `README.md`, `about.html`, `notice.html`, `epl-v20`, `edl-v10`, and source-file license notices.
