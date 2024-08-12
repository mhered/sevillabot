# Motor Wiring

Note: kept L298N jumper on instead of piggybacking 5V from Arduino

![](./assets/wiring_diagram_2.png)

| L298 | Arduino | Motor                  | Color  |
| ---- | ------- | ---------------------- | ------ |
| out1 |         | motor B (left) red     | red    |
| out2 |         | motor B (left) white   | white  |
| out3 |         | motor A (right) white  | white  |
| out4 |         | motor A (right) red    | red    |
| n1   | D10     |                        | brown  |
| n2   | D6      |                        | purple |
| n3   | D9      |                        | grey   |
| n4   | D5      |                        | orange |
|      | D3      | motor B (left) yellow  | yellow |
|      | D2      | motor B (left) green   | green  |
|      | A5      | motor A (right) yellow | yellow |
|      | A4      | motor A (right) green  | green  |

