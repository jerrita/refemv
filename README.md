# Refemv RV32I

- This is a simple CPU implements risc-v 32i.
- It is just my refine product and notes of FemtoRV learning.
- Use on your own risk.


## Design

- CPU Design
    - [x] Multi-cycled CPU
    - [x] Memory-Mapped IO (partial)

- [ ] Boot Sequential
    1. Hard-coded boot code (in bram).
    2. Copy program from flash to spram.
    3. Jump to spram and execute.



## Reference
- https://github.com/damdoy/ice40_ultraplus_examples
- https://projectf.io/posts/spram-ice40-fpga