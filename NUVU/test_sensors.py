while True:
    print([read_adc(ch) for ch in ADC_CHANNELS])
    time.sleep(1)
