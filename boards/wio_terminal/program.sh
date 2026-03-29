# running on squarenuc1tb be like:
# Pre-contition:
#   export PATH=~/programs/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/:$PATH

CARGO_TARGET=${1}
ELF_FILE=${2}
GDB_PORT=${3}
echo arm-none-eabi-objcopy -I elf32-littlearm -O binary $(readlink -f ${ELF_FILE}) ${CARGO_TARGET}
# cargo build --example ${CARGO_TARGET} &&
arm-none-eabi-objcopy -I elf32-littlearm -O binary $(readlink -f ${ELF_FILE}) ${CARGO_TARGET}.bin &&
  gdb-multiarch -ex "target remote localhost:${GDB_PORT}" -ex "mon reset halt" -ex "mon program $(readlink -f ${CARGO_TARGET}.bin) verify 0x00000" -ex "q" &&
  /home/cooler1989/programs/cgdb/cgdb/cgdb -d arm-none-eabi-gdb ${ELF_FILE} -ex "target remote localhost:${GDB_PORT}" -ex "mon reset halt"
# -ex "mon arm semihosting enable" -ex "mon arm semihosting_redirect tcp 2499"

# Use with openocd command:
# /home/cooler1989/programs/openocd/src/openocd -s /home/cooler1989/programs/openocd/tcl/ -f interface/jlink.cfg -c "transport select swd" -f target/atsame5x.cfg -c "adapter speed 5000"
