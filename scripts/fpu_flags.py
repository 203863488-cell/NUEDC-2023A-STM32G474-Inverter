Import("env")

# STM32G474 contains an FPv4-SP-D16 FPU.  PlatformIO's generic stm32cube
# environment only selects Cortex-M4 by default, which leaves floating point
# operations implemented by slow __aeabi_* software helpers.  softfp keeps the
# existing ABI while allowing the compiler to emit hardware VFP instructions.
fpu_flags = [
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=softfp",
]

env.AppendUnique(LINKFLAGS=fpu_flags)
