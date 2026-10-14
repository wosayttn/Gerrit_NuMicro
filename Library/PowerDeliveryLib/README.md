# M2L31 USB PD Library README

**PD Library Version: V1.0**  
**Document Revision:** 1.0
**Issue Date:** 2026-09-15  
**Purpose:** Customer package quick integration guide

## 1. Purpose and Scope

This package provides the M2L31 USB Power Delivery (PD) Library as binary libraries with public headers and customer documentation. Library source code is not delivered.

Use of this package is governed by [`LICENSE.md`](LICENSE.md). The license controls all permitted use, distribution, confidentiality, ownership, warranty, and liability terms. This README is an integration guide and does not replace or modify the license or another applicable signed agreement.

Portions of the USB PD Library are derived from the ChromiumOS EC project. See `License/ChromiumOS_EC_LICENSE.md` for the applicable license terms.

## 2. Select One Library Variant

| Variant | Intended role | Integration note |
|---|---|---|
| `PD_SRC` | Source | Delivered binary variant; outside the 2026 reference-platform verification context. |
| `PD_SNK` | Sink | Included in the 2026 reference-platform verification context. |
| `PD_DRP` | Dual-Role Power | Included in the 2026 reference-platform verification context. |

Link only the variant selected for the product. Do not transfer reference-platform observations from `PD_DRP` or `PD_SNK` to `PD_SRC` or to another product configuration.

## 3. Integration Prerequisites and Responsibilities

The customer application and product design own:

- Source and Sink PDO tables and any enabled APDO configuration;
- the power path, VBUS enable/disable sequencing, discharge, regulation, sensing, and board protection;
- required library callbacks, application policy, command handling, and fault recovery;
- scheduler or service-loop integration and required periodic processing;
- connector, cable, current, power, and thermal policy; and
- final hardware, firmware, protocol, electrical, safety, and product validation.

The TCPC is integrated in M2L31; an external TCPC is not applicable to this integration.

## 4. Controlled Configurable Envelope and Reference PDOs

The controlled Library-configurable envelope is:

- Source: up to **20 V/3 A**
- Sink: up to **20 V/5 A**

These values describe Library configuration capability only. They are not a product electrical rating and are not a result of the current reference-baseline validation. Advertise or request a PDO only after validating the product power path, protection, connector, cable, and thermal envelope.

The current reference configuration is not a general product setting:

- `PD_DRP` Source Fixed PDOs: **5 V/3 A, 9 V/1.5 A, 15 V/1 A**
- `PD_DRP` Sink Fixed PDO: **5 V/3 A**
- `PD_SNK` Sink Fixed PDOs: **5 V/3 A, 9 V/3 A, 15 V/3 A, 20 V/3 A**

The reference PDO inputs require release-record provenance before they are treated as a reproducible delivered configuration.

## 5. Operating Integration Boundaries

- `PD_DRP` uses standard/general role alternation. `Try.SRC` and `Try.SNK` are not supported.
- The DRP role enables DRP toggling. The SRC role defaults to a fixed Rp-3A setting, while the SNK role uses a fixed Rd setting.
- `PR_SWAP` and `DR_SWAP` are available only with `PD_DRP` and depend on application callbacks, board power-path behavior, and end-to-end product integration. Availability is not a product validation PASS.
- VCONN and Cable Identity are not enabled in the reference baseline. No e-marker discovery or 5 A cable behavior is claimed.
- PPS Source remains verification pending, and no Source PPS APDO is active in the reference baseline.
- The reference OCP strategy is external upper-layer monitoring: INA219/shunt-based current monitoring is polled every **10 ms**; the threshold is the negotiated PDO current plus **100 mA**; an exceedance initiates **Hard Reset -> VBUS off -> renegotiation**. This is not native Library protection, a hardware cutoff claim, or a validated precision-current claim.
- OVP is not included in the current reference integration. The final product must provide and validate protection appropriate to its architecture and requirements.



## 6. Known Limitations and Certification Boundary

- This package does not constitute or imply USB-IF certification, assignment of a USB-IF Test ID (TID), product qualification, safety approval, interoperability approval, or any electrical, power, or thermal rating for the final product. Qualification and certification are applicable only to the customer's final product configuration, including the hardware, firmware, power architecture, protection circuitry, connector, cable policy, PDO/APDO configuration, and enabled features.
- Library source code is not provided. Product-specific integration, configuration, enablement, validation, and certification remain the customer's responsibility unless otherwise agreed in writing.
- For reference purposes, in-house testing using the Teledyne LeCroy M310e has confirmed that the tested SNK and DRP reference configurations pass the applicable USB PD CTS and USB Type-C functional test cases. These results apply only to the tested reference configurations and do not constitute USB-IF certification or product qualification.
