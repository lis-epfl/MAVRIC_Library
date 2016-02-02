mkdir avr32-gcc
cd avr32-gcc

# Get archives
wget https://gist.github.com/jlecoeur/d83107dda77c4bd62a66/raw/1dc9c3efe832db7cfcd86694693f620d811e9802/avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
wget https://gist.github.com/jlecoeur/d83107dda77c4bd62a66/raw/471a24180afc9c799848e017aa2f8cf10c55cb1a/atmel-headers-6.1.3.1475.zip

# Unpack archives
tar xvfz avr32-gnu-toolchain-3.4.2.435-linux.any.x86.tar.gz
unzip atmel-headers-6.1.3.1475.zip

# Install headers and binaries
sudo mv avr32-gnu-toolchain-linux_x86 /usr/local
sudo mv atmel-headers-6.1.3.1475/* /usr/local/avr32-gnu-toolchain-linux_x86/avr32/include

# Create symbolic links
sudo ln -s /usr/local/avr32-gnu-toolchain-linux_x86/bin/avr32* /usr/local/bin

# Print avr32-gcc version
avr32-gcc -v