# Memory Map for M3351

**Note:** Alias regions means that secure and non-secure addresses are mapped to the same physical memory.

| Memory Region Name | Base Addr   | Size        | IDAU | Description                       |
| :----------------- | :---------- | :---------- | :--- | :-------------------------------- |
| SRAM01             | 0x2000_0000 | 0x0002_0000 | S    | 2 banks of 64 KB                  |
| SRAM01             | 0x3000_0000 | 0x0002_0000 | NS   | Non-secure alias for SRAM01       |
| SRAM2              | 0x2002_0000 | 0x0000_8000 | S    | 32 KB (Need to disable SRAM ECC)  |
| SRAM2              | 0x3002_0000 | 0x0000_8000 | NS   | Non-secure alias for SRAM2        |
| APROM              | 0x0000_0000 | 0x0010_0000 | S    | 1 MB                              |
| APROM              | 0x1000_0000 | 0x0010_0000 | NS   | Non-secure alias for APROM        |
| LDROM              | 0x0F10_0000 | 0x0000_4000 | S    | 16 KB (Secure only)               |
| Data Flash 0       | 0x0F20_0000 | 0x0000_4000 | S    | 16 KB (Secure only)               |
| Data Flash 1       | 0x0F21_0000 | 0x0001_0000 | S    | 64 KB                             |
| Data Flash 1       | 0x1F21_0000 | 0x0001_0000 | NS   | Non-secure alias for Data Flash 1 |
| EBI                | 0x6000_0000 | 0x0030_0000 | S    | 3 MB                              |
| EBI                | 0x7000_0000 | 0x0030_0000 | NS   | Non-secure alias for EBI          |
| Aliased SRAM01     | 0x0080_0000 | 0x0002_0000 | S    | 2 banks of 64 KB                  |
| Aliased SRAM01     | 0x1080_0000 | 0x0002_0000 | NS   | Non-secure alias for SRAM01       |
| Aliased SRAM2      | 0x0082_0000 | 0x0000_8000 | S    | 32 KB (Need to disable SRAM ECC)  |
| Aliased SRAM2      | 0x1082_0000 | 0x0000_8000 | NS   | Non-secure alias for SRAM2        |
| Peripherals        | 0x4000_0000 | 0x1000_0000 | S    | Secure Peripherals                |
| Peripherals        | 0x5000_0000 | 0x1000_0000 | NS   | Non-secure alias for Peripherals  |
