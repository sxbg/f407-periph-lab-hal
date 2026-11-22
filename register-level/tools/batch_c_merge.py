"""Batch-C merge: comms peripherals into f407-periph-lab.ioc (one-shot tool)."""
from pathlib import Path

SRC = Path(__file__).parent / "f407-periph-lab.ioc"

src = SRC.read_text(encoding="utf-8")
K = {}
for l in src.replace("\r\n", "\n").strip().split("\n"):
    if "=" in l and not l.startswith("#"):
        k, v = l.split("=", 1)
        K[k] = v
print("base keys:", len(K), "| IPNb:", K.get("Mcu.IPNb"))

# new IPs (IPNb 11 -> 20)
for i, ip in enumerate(["USART2", "USART3", "I2C1", "SPI1", "I2S2", "CAN1",
                        "SDIO", "USB_OTG_FS", "FSMC"], start=11):
    K[f"Mcu.IP{i}"] = ip
K["Mcu.IPNb"] = "20"

# sections (minimal; empirical convergence on drop)
K["USART2.IPParameters"] = "VirtualMode,BaudRate"
K["USART2.VirtualMode"] = "VM_ASYNC"
K["USART2.BaudRate"] = "115200"
K["USART3.IPParameters"] = "VirtualMode,BaudRate"
K["USART3.VirtualMode"] = "VM_ASYNC"
K["USART3.BaudRate"] = "115200"
K["I2C1.ClockSpeed"] = "100000"
K["I2C1.IPParameters"] = "ClockSpeed"
K["SPI1.IPParameters"] = "Mode"
K["SPI1.Mode"] = "SPI_MODE_MASTER"
K["I2S2.AudioFreq"] = "I2S_AUDIOFREQ_48K"
K["I2S2.IPParameters"] = "AudioFreq,VirtualMode"
K["I2S2.VirtualMode"] = "I2S_MODE_MASTER_TX"
K["CAN1.IPParameters"] = "Prescaler"
K["CAN1.Prescaler"] = "6"
K["USB_OTG_FS.IPParameters"] = "VirtualMode"
K["USB_OTG_FS.VirtualMode"] = "Device_Mode"

newpins = [
    ("PA2", "USART2_TX", "LOG2_TX"), ("PA3", "USART2_RX", "LOG2_RX"),
    ("PB10", "USART3_TX", "RS485_TX"), ("PB11", "USART3_RX", "RS485_RX"),
    ("PB6", "I2C1_SCL", "EE_SCL"), ("PB7", "I2C1_SDA", "EE_SDA"),
    ("PA5", "SPI1_SCK", "FLASH_SCK"), ("PA6", "SPI1_MISO", "FLASH_MISO"),
    ("PA7", "SPI1_MOSI", "FLASH_MOSI"),
    ("PB12", "I2S2_WS", "I2S_WS"), ("PB13", "I2S2_CK", "I2S_CK"),
    ("PB15", "I2S2_SD", "I2S_SD"),
    ("PB8", "CAN1_RX", "CAN_RX"), ("PB9", "CAN1_TX", "CAN_TX"),
    ("PC8", "SDIO_D0", "TF_D0"), ("PC9", "SDIO_D1", "TF_D1"),
    ("PC10", "SDIO_D2", "TF_D2"), ("PC11", "SDIO_D3", "TF_D3"),
    ("PC12", "SDIO_CK", "TF_CK"), ("PD2", "SDIO_CMD", "TF_CMD"),
    ("PA11", "USB_DM", "USB_DM"), ("PA12", "USB_DP", "USB_DP"),
]
fsmc = [
    ("PD14", "FSMC_D0", "LCD_D0"), ("PD15", "FSMC_D1", "LCD_D1"),
    ("PD0", "FSMC_D2", "LCD_D2"), ("PD1", "FSMC_D3", "LCD_D3"),
    ("PE7", "FSMC_D4", "LCD_D4"), ("PE8", "FSMC_D5", "LCD_D5"),
    ("PE9", "FSMC_D6", "LCD_D6"), ("PE10", "FSMC_D7", "LCD_D7"),
    ("PE11", "FSMC_D8", "LCD_D8"), ("PE12", "FSMC_D9", "LCD_D9"),
    ("PE13", "FSMC_D10", "LCD_D10"), ("PE14", "FSMC_D11", "LCD_D11"),
    ("PE15", "FSMC_D12", "LCD_D12"), ("PD8", "FSMC_D13", "LCD_D13"),
    ("PD9", "FSMC_D14", "LCD_D14"), ("PD10", "FSMC_D15", "LCD_D15"),
    ("PD4", "FSMC_NOE", "LCD_RD"), ("PD5", "FSMC_NWE", "LCD_WE"),
    ("PD11", "FSMC_A16", "LCD_RS"), ("PD7", "FSMC_NE1", "LCD_CS"),
    ("PA15", "GPIO_Output", "LCD_BL"),
]
allpins = newpins + fsmc

existing_hw = [v for k, v in K.items() if k.startswith("Mcu.Pin")
               and k != "Mcu.PinsNb" and not v.startswith("VP_")]
existing_vps = [v for k, v in K.items() if k.startswith("Mcu.Pin")
                and v.startswith("VP_")]
order = existing_hw + [p[0] for p in allpins] + existing_vps

for k in list(K):
    if k.startswith("Mcu.Pin") and k != "Mcu.PinsNb":
        K.pop(k)
for n, p in enumerate(order):
    K[f"Mcu.Pin{n}"] = p
K["Mcu.PinsNb"] = str(len(order))

for pin, sig, label in allpins:
    K[f"{pin}.Signal"] = sig
    K[f"{pin}.Locked"] = "true"
    if sig.startswith("USART"):
        K[f"{pin}.Mode"] = "Asynchronous"
        K[f"{pin}.GPIOParameters"] = "GPIO_Label"
        K[f"{pin}.GPIO_Label"] = label
    elif sig == "GPIO_Output":
        K[f"{pin}.GPIOParameters"] = "GPIO_Speed,PinState,GPIO_Label"
        K[f"{pin}.GPIO_Speed"] = "GPIO_SPEED_FREQ_LOW"
        K[f"{pin}.PinState"] = "GPIO_PIN_RESET"
        K[f"{pin}.GPIO_Label"] = label
    elif sig.startswith(("I2S", "I2C", "SDIO", "FSMC", "CAN", "SPI")):
        K[f"{pin}.GPIOParameters"] = "GPIO_Label"
        K[f"{pin}.GPIO_Label"] = label
    # USB DM/DP: signal+locked only

lines = [f"{k}={v}" for k, v in K.items()]
lines.sort(key=lambda l: l.split("=", 1)[0].encode())
data = ("#MicroXplorer Configuration settings - do not modify\r\n"
        + "\r\n".join(lines) + "\r\nboard=custom\r\n").encode("ascii")
SRC.write_bytes(data)
print("batch-C written", len(data), "bytes | total pins", len(order))
