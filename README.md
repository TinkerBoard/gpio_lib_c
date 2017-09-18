# GPIO_Lib_C
  GPIO_LIB is a extension of WiringPi, it can control low speed peripherial of Tinker Board.
  
# Getting Source Code
1. make sure network is work fine.
2. install git
    ```shell
    sudo apt-get update
    sudo apt-get install git
    ```
3. download library from git
    ```shell
    git clone https://github.com/TinkerBoard/gpio_lib_c.git
    ```

# How to Build
```
cd gpio_lib_c
chmod a+x build
sudo ./build
```
It will build specific project you define in build. the default folder are `gpio`, `example`,...
if you create new c file, you need to modify Makefile to compile it.

If you want to build it by gcc directly, you need to add flag `-DTINKER_BOARD`.
  
# Usage
 `gpio readall`
 ```
 +-----+-----+---------+------+---+--Tinker--+---+------+---------+-----+-----+
 | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 | 252 |   8 |   SDA.1 |      | 1 |  3 || 4  |   |      | 5V      |     |     |
 | 253 |   9 |   SCL.1 |      | 1 |  5 || 6  |   |      | 0v      |     |     |
 |  17 |   7 | GPIO0C1 |   IN | 1 |  7 || 8  | 1 | SERL | TxD1    | 15  | 161 |
 |     |     |      0v |      |   |  9 || 10 | 1 | SERL | RxD1    | 16  | 160 |
 | 164 |   0 | GPIO5B4 |   IN | 1 | 11 || 12 | 0 |      | GPIO6A0 | 1   | 184 |
 | 166 |   2 | GPIO5B6 | SERL | 1 | 13 || 14 |   |      | 0v      |     |     |
 | 167 |   3 | GPIO5B7 | SERL | 1 | 15 || 16 | 1 | IN   | GPIO5B2 | 4   | 162 |
 |     |     |    3.3v |      |   | 17 || 18 | 1 | IN   | GPIO5B3 | 5   | 163 |
 | 257 |  12 |   MOSI1 |      | 0 | 19 || 20 |   |      | 0v      |     |     |
 | 256 |  13 |   MISO1 |      | 1 | 21 || 22 | 0 | IN   | GPIO5C3 | 6   | 171 |
 | 254 |  14 |   SCLK1 |      | 1 | 23 || 24 | 1 |      | CE0     | 10  | 255 |
 |     |     |      0v |      |   | 25 || 26 | 1 |      | CE1     | 11  | 251 |
 | 233 |  30 |   SDA.2 |   IN | 1 | 27 || 28 | 1 |      | SCL.2   | 31  | 234 |
 | 165 |  21 | GPIO5B5 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 | 168 |  22 | GPIO5C0 |   IN | 1 | 31 || 32 | 1 | PWM  | GPIO7C7 | 26  | 239 |
 | 238 |  23 | GPIO7C6 |  PWM | 1 | 33 || 34 |   |      | 0v      |     |     |
 | 185 |  24 | GPIO6A1 |      | 0 | 35 || 36 | 1 | IN   | GPIO7A7 | 27  | 223 |
 | 224 |  25 | GPIO7B0 |   IN | 0 | 37 || 38 | 1 |      | GPIO6A3 | 28  | 187 |
 |     |     |      0v |      |   | 39 || 40 | 0 |      | GPIO6A4 | 29  | 188 |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | CPU | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | CPU |
 +-----+-----+---------+------+---+--Tinker--+---+------+---------+-----+-----+
```
# Troubleshooting
If you meet SSL issue as below 
> "fatal: unable to access 'https://github.com/TinkerBoard/gpio_lib_c.git': server certificate verification failed. CAfile: /etc/ssl/certs/ca-certificates.crt CRLfile: none"
  
short-term solution is `export GIT_SSL_NO_VERIFY=1`.
# More Information
  You can refer to URL.
