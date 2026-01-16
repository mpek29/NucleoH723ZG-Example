# FDCAN-NUCLEO-H723ZG

## CAN Message Mapping

### Cyclic Counter & FCPM State
| Field           | Byte | Bits         |
|-----------------|------|--------------|
| Cyclic Counter  | 0    | 0-3          |
| FCPM State      | 0    | 4-7          |

### Tank Level [bar.g]
| Field                | Byte | Bits         | 
|----------------------|------|--------------|
| Tank Level [bar.g]   | 1    | 0-7          |
| Tank Level [bar.g]   | 2    | 0-1          |

### Hybrid Battery Voltage [V]
| Field                      | Byte | Bits         |
|----------------------------|------|--------------|
| Hybrid Battery Voltage [V] | 2    | 2-7          |
| Hybrid Battery Voltage [V] | 3    | 0-3          |


### Unit in Fault
| Field         | Byte | Bits         |
|---------------|------|--------------|
| Unit in fault | 3    | 4-7          |

### SM Input Power [W]
| Field                    | Byte | Bits         |
|--------------------------|------|--------------|
| SM Input Power [W]       | 4    | 6-7          |
| SM Input Power [W]       | 5    | 0-7          |

### Hybrid Battery Power [W]
| Field                    | Byte | Bits         |
|--------------------------|------|--------------|
| Hybrid Battery Power [W] | 6    | 0-7          |
| Hybrid Battery Power [W] | 7    | 0-1          |

### Error Code
| Field      | Byte       | Bits         |
|------------|------------|--------------|
| Error code | no byte    | 3-7          |

## Scaling and Offset for Non-Binary Values
| Variable Name                | Scale (units/bit) | Offset (units) | Bits | Maximum Value |
|------------------------------|-------------------|----------------|------|--------------|
| Tank Level [bar.g]           | 0.5               | 0              | 10   | 511.5        |
| Hybrid Battery Voltage [V]   | 0.1               | 0              | 10   | 102.3        |
| Output Power [W]             | 10                | 0              | 10   | 10230        |
| SM Input Power [W]           | 5                 | 0              | 10   | 5115         |
| Hybrid Battery Power [W]     | 10                | 5000           | 10   | 10230        |
