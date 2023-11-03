"""
    01280a1904 (800x600, 1056x628, 40 MHz, hpol-p-vpol-p)
    01291a2904 (1024x768, 1344x806, 65 MHz, hpol-n-vpol-n)
    01270a1e02 (1024x768, 1344x806, 65 MHz, hpol-p-vpol-n)
    012d213204 (1280x720, 1664x748, 74 MHz, hpol-p-vpol-p)
    012d183202 (1280x1024, 1688x1066, 108 MHz, hpol-p-vpol-p)
    0131112302 (1680x1050, 1840x1080, 119 MHz, hpol-n-vpol-p)
    012d243202 (1600x1200, 2160x1250, 162 MHz, hpol-p-vpol-p)
    012c1b2802 (1920x1080, 2200x1125, 148 MHz, hpol-p-vpol-p)
"""
pll1 = 0x01, 0x28, 0x0a, 0x19, 0x04
pll2 = 0x01, 0x29, 0x1a, 0x29, 0x04
pll3 = 0x01, 0x27, 0x0a, 0x1e, 0x02
pll4 = 0x01, 0x2d, 0x21, 0x32, 0x04
pll5 = 0x01, 0x2d, 0x18, 0x32, 0x02
pll6 = 0x01, 0x31, 0x11, 0x23, 0x02
pll7 = 0x01, 0x2d, 0x24, 0x32, 0x02
pll8 = 0x01, 0x2c, 0x1b, 0x28, 0x02

plls = [pll1, pll2, pll3, pll4, pll5, pll6, pll7, pll8]
freqs = [40, 65, 65, 74, 108, 119, 162, 148]

minvar = 100000000
mini = 0
minfreqs = []
for i in range(2**len(pll1)):
    generatedfreq = []
    for freqnum, freq in enumerate(freqs):
        for j in range(len(pll1)):
            if i & (1 << j):
                freq *= plls[freqnum][j]
            else:
                freq /= plls[freqnum][j]
        generatedfreq.append(freq)

    # quick variance calculation
    variance = (max(generatedfreq) - min(generatedfreq)) / sum(generatedfreq) * len(generatedfreq)

    if variance < minvar:
        minvar = variance
        mini = i
        minfreqs = generatedfreq

print(minvar, bin(mini), minfreqs)

# Outputs:
#0.003369839932603475 0b11000 [10.0, 10.000000000000002, 10.000000000000002, 9.966329966329965, 10.0, 10.0, 10.0, 9.966329966329967]

# Probably 10 MHz with PLL registers as [unknown, mul1, mul2, div1, div2]