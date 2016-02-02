mkdir avr32-gcc
cd avr32-gcc
wget https://gist.github.com/jlecoeur/d83107dda77c4bd62a66/raw/1dc9c3efe832db7cfcd86694693f620d811e9802/avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
tar -xf avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
sudo cp avr32-gnu-toolchain-linux_x86/bin/* /usr/local/bin/