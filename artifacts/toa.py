from math import ceil

# Design parameters

IH=1    # 1 for no header is present (implicit mode)
DE=1    # 1 when LowDataRateOptimize=1
CRC=1   # 1 if CRC is present

PL=8    # payload length in bytes (1 to 255)
# SF=12   # Spreading factor 6-12
# BW=12500 # bandwidth
# CR=4    # 1 for 4/5, 4 for 4/8 (Coding rate)



# Possible LoRa Bandwidth values in Hz
BW_values = [7800, 10400, 15600, 20800, 31200, 41700, 62500, 125000, 250000, 500000]

# Loop through all combinations of SF, BW, and CR
for SF in range(6, 13):  # SF is an integer in [6, 12]
    for BW in BW_values:  # BW is from the set of frequencies in Hz
        for CR in range(1, 5):  # CR is an integer in [1, 4]
            # Lora symbol rate (data rate)
            R_s = BW / 2 ** SF
            # Symbol period
            T_s = 1 / R_s
            # Number of symbols to send
            n_payload = 8 + max(
                ceil((8 * PL - 4 * SF + 28 + 16 * CRC - 20 * IH) / (4 * (SF - 2 * DE))) * (CR + 4),
                0
            )
            # Payload duration
            T_payload = n_payload * T_s
            
            # Print results for the current combination
            print(f"Lora Symbol period: {T_s * 5}, SF: {SF}, BW: {BW} Hz, CR: 4/{4 + CR}, T_payload: {T_payload * 100.0:.6f} ms")
