if [ ! -f "sd.bin" ]; then
qemu-img create -f raw sd.bin 64M
mkfs.vfat sd.bin
fi


# 第二个端口是和mavros通讯， mavros和飞控在一个设备上，所以应该是前后都是local
# qemu-system-arm -M vexpress-a9 -kernel build/fmt_qemu-vexpress-a9.bin -display none -sd sd.bin -serial stdio -serial udp:localhost:14550@localhost:14551 -serial udp:localhost:14551@localhost:14554
qemu-system-arm -M vexpress-a9 -kernel build/fmt_qemu-vexpress-a9.bin -display none -sd sd.bin -serial stdio -serial udp:192.168.1.115:14553@192.168.1.114:14552 -serial udp:localhost:14551@localhost:14554

