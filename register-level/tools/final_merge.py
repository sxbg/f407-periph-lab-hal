"""Final IOC merge for f407-periph-lab: proven-format keys only.

IOC scope (all proven formats):
  IPs: NVIC RCC SYS TIM1 TIM2 TIM3 TIM4 TIM5 TIM8 TIM9 USART1 USART2 USART3
       CAN1 USB_OTG_FS USB_DEVICE
  Adapter-owned (OUT of IOC, pins reserved as GPIO_Analog/GPIO with labels,
  bsp modules own registers; documented in docs/periph/*.md):
       ADC1-3, DAC, DMA1/2, SPI1(W25Q), I2C1(24C02), I2S2, SDIO(TF), FSMC(LCD),
       TIM6/7, RTC, IWDG, WWDG, CRC, RNG, PWR, FlashIAP, EXTI, DWT
"""
from pathlib import Path

SRC = Path(__file__).parent / "f407-periph-lab.ioc"

K = {}
for l in SRC.read_text(encoding="utf-8").replace("\r\n", "\n").strip().split("\n"):
    if "=" in l and not l.startswith("#"):
        k, v = l.split("=", 1)
        K[k] = v
print("base keys:", len(K), "| IPNb:", K.get("Mcu.IPNb"))

# ---- purge all Mcu.IPn / pin keys, rebuild from scratch ----
for k in list(K):
    if k.startswith("Mcu.IP") or k.startswith("Mcu.Pin"):
        K.pop(k)

IPS = ["NVIC", "RCC", "SYS", "TIM1", "TIM2", "TIM3", "TIM4", "TIM5", "TIM8",
       "TIM9", "USART1", "USART2", "USART3", "CAN1", "USB_OTG_FS", "USB_DEVICE"]
for i, ip in enumerate(IPS):
    K[f"Mcu.IP{i}"] = ip
K["Mcu.IPNb"] = str(len(IPS))

# ---- CAN1 section (from D:\ClaudeCodeWorkSpace\USB F407.ioc CAN2 template) ----
K["CAN1.BS1"] = "CAN_BS1_8TQ"
K["CAN1.BS2"] = "CAN_BS2_3TQ"
K["CAN1.IPParameters"] = "CalculateTimeQuantum,CalculateTimeBit,CalculateBaudRate,Prescaler,BS1,BS2"
K["CAN1.Prescaler"] = "6"

# ---- USB_OTG_FS + USB_DEVICE (adapted to FS/CDC from same template) ----
K["USB_OTG_FS.IPParameters"] = "VirtualMode-Device_Only"
K["USB_OTG_FS.VirtualMode-Device_Only"] = "Device_Only"
K["USB_DEVICE.CLASS_NAME_FS"] = "CDC"
K["USB_DEVICE.IPParameters"] = "VirtualMode-CDC_FS,VirtualModeFS,CLASS_NAME_FS"
K["USB_DEVICE.VirtualMode-CDC_FS"] = "Cdc"
K["USB_DEVICE.VirtualModeFS"] = "Cdc_FS"

# ---- TIM sections carried over (batch-B survivors) ----
K["TIM1.Channel-PWM\\ Generation1\\ CH1"] = "TIM_CHANNEL_1"
K["TIM1.IPParameters"] = "Channel-PWM Generation1 CH1,Prescaler"
K["TIM1.Prescaler"] = "0"
K["TIM2.Channel-Input\\ Capture2\\ from\\ TI2"] = "TIM_CHANNEL_2"
K["TIM2.IPParameters"] = "Channel-Input Capture2 from TI2,Prescaler"
K["TIM2.Prescaler"] = "0"
K["TIM3.EncoderMode"] = "TIM_ENCODERMODE_TI12"
K["TIM3.IPParameters"] = "EncoderMode,AutoReloadPreload,Period"
K["TIM3.AutoReloadPreload"] = "TIM_AUTORELOAD_PRELOAD_ENABLE"
K["TIM3.Period"] = "65535"
K["TIM3.Prescaler"] = "0"
K["TIM4.Channel-PWM\\ Generation1\\ CH1"] = "TIM_CHANNEL_1"
K["TIM4.IPParameters"] = "Channel-PWM Generation1 CH1,Prescaler"
K["TIM4.Prescaler"] = "0"
for t in ("TIM5", "TIM8", "TIM9"):
    K[f"{t}.IPParameters"] = "Prescaler"
    K[f"{t}.Prescaler"] = "0"
K["TIM8.Channel-PWM\\ Generation1\\ CH1"] = "TIM_CHANNEL_1"
K["TIM8.Channel-PWM\\ Generation2\\ CH2"] = "TIM_CHANNEL_2"
K["TIM8.Channel-PWM\\ Generation2\\ CH2N"] = "TIM_CCxN_ENABLE"
K["TIM8.IPParameters"] = ("Channel-PWM Generation1 CH1,Channel-PWM Generation2 CH2,"
                          "Channel-PWM Generation2 CH2N,Prescaler")
K["TIM8.Prescaler"] = "0"

# ---- USART sections ----
for u in ("USART1", "USART2", "USART3"):
    K[f"{u}.IPParameters"] = "VirtualMode,BaudRate"
    K[f"{u}.VirtualMode"] = "VM_ASYNC"
    K[f"{u}.BaudRate"] = "115200"

# ---- full 82-pin map ----
# (pin, signal, label, kind)  kind: auto|out|in|analog
# auto = signal+locked only; out/in/analog = GPIO with companion keys
PINS = [
    # GPIOA
    ("PA0-WKUP", "GPIO_Input", "KEY1", "in", "GPIO_PuPd,GPIO_Label", "GPIO_PULLDOWN"),
    ("PA1", "GPIO_Input", "KEY2", "in", "GPIO_PuPd,GPIO_Label", "GPIO_PULLUP"),
    ("PA2", "USART2_TX", "LOG2_TX", "auto", None, None),
    ("PA3", "USART2_RX", "LOG2_RX", "auto", None, None),
    ("PA4", "GPIO_Analog", "DAC_OUT1", "analog", None, None),
    ("PA5", "GPIO_Analog", "FLASH_SCK", "analog", None, None),
    ("PA6", "GPIO_Analog", "FLASH_MISO", "analog", None, None),
    ("PA7", "GPIO_Analog", "FLASH_MOSI", "analog", None, None),
    ("PA8", "TIM1_CH1", "TIM1_CH1", "auto", None, None),
    ("PA9", "USART1_TX", "LOG_TX", "auto", None, None),
    ("PA10", "USART1_RX", "LOG_RX", "auto", None, None),
    ("PA11", "USB_OTG_FS_DM", "USB_DM", "auto", None, None),
    ("PA12", "USB_OTG_FS_DP", "USB_DP", "auto", None, None),
    ("PA13", "SYS_JTMS-SWDIO", "SWDIO", "auto", None, None),
    ("PA14", "SYS_JTCK-SWCLK", "SWCLK", "auto", None, None),
    ("PA15", "GPIO_Output", "LCD_BL", "out", "GPIO_Speed,PinState,GPIO_Label", "RESET"),
    # GPIOB
    ("PB0", "GPIO_Analog", "ADC1_IN8", "analog", None, None),
    ("PB1", "GPIO_Analog", "ADC2_IN9", "analog", None, None),
    ("PB2", "GPIO_Output", "LED_BLUE", "out", "GPIO_Speed,PinState,GPIO_Label", "SET"),
    ("PB3", "TIM2_CH2", "TIM2_CH2", "auto", None, None),
    ("PB4", "TIM3_CH1", "ENC_A", "auto", None, None),
    ("PB5", "TIM3_CH2", "ENC_B", "auto", None, None),
    ("PB6", "GPIO_Analog", "EE_SCL", "analog", None, None),
    ("PB7", "GPIO_Analog", "EE_SDA", "analog", None, None),
    ("PB8", "GPIO_Analog", "CAN_RX", "analog", None, None),
    ("PB9", "GPIO_Analog", "CAN_TX", "analog", None, None),
    ("PB10", "USART3_TX", "RS485_TX", "auto", None, None),
    ("PB11", "USART3_RX", "RS485_RX", "auto", None, None),
    ("PB12", "GPIO_Analog", "I2S_WS", "analog", None, None),
    ("PB13", "GPIO_Analog", "I2S_CK", "analog", None, None),
    ("PB14", "TIM8_CH2N", "TIM8_CH2N", "auto", None, None),
    ("PB15", "GPIO_Analog", "I2S_SD", "analog", None, None),
    # GPIOC
    ("PC0", "GPIO_Analog", "ADC1_IN10", "analog", None, None),
    ("PC1", "GPIO_Analog", "ADC2_IN11", "analog", None, None),
    ("PC2", "GPIO_Analog", "ADC3_IN12", "analog", None, None),
    ("PC3", "GPIO_Analog", "ADC3_IN13", "analog", None, None),
    ("PC4", "GPIO_Analog", "ADC1_IN14", "analog", None, None),
    ("PC5", "GPIO_Output", "LED_GREEN", "out", "GPIO_Speed,PinState,GPIO_Label", "RESET"),
    ("PC6", "TIM8_CH1", "TIM8_CH1", "auto", None, None),
    ("PC7", "TIM8_CH2", "TIM8_CH2", "auto", None, None),
    ("PC8", "GPIO_Analog", "TF_D0", "analog", None, None),
    ("PC9", "GPIO_Analog", "TF_D1", "analog", None, None),
    ("PC10", "GPIO_Analog", "TF_D2", "analog", None, None),
    ("PC11", "GPIO_Analog", "TF_D3", "analog", None, None),
    ("PC12", "GPIO_Analog", "TF_CK", "analog", None, None),
    ("PC13-ANTI_TAMP", "GPIO_Output", "FLASH_CS", "out", "GPIO_Speed,PinState,GPIO_Label", "SET"),
    ("PC14-OSC32_IN", "RCC_OSC32_IN", "OSC32_IN", "auto", None, None),
    ("PC15-OSC32_OUT", "RCC_OSC32_OUT", "OSC32_OUT", "auto", None, None),
    # GPIOD
    ("PD0", "GPIO_Analog", "LCD_D2", "analog", None, None),
    ("PD1", "GPIO_Analog", "LCD_D3", "analog", None, None),
    ("PD2", "GPIO_Analog", "TF_CMD", "analog", None, None),
    ("PD3", "GPIO_Analog", "EXP_PD3", "analog", None, None),
    ("PD4", "GPIO_Analog", "LCD_RD", "analog", None, None),
    ("PD5", "GPIO_Analog", "LCD_WE", "analog", None, None),
    ("PD6", "GPIO_Analog", "EXP_PD6", "analog", None, None),
    ("PD7", "GPIO_Analog", "LCD_CS", "analog", None, None),
    ("PD8", "GPIO_Analog", "LCD_D13", "analog", None, None),
    ("PD9", "GPIO_Analog", "LCD_D14", "analog", None, None),
    ("PD10", "GPIO_Analog", "LCD_D15", "analog", None, None),
    ("PD11", "GPIO_Analog", "LCD_RS", "analog", None, None),
    ("PD12", "TIM4_CH1", "TIM4_CH1", "auto", None, None),
    ("PD13", "GPIO_Analog", "T_CS", "analog", None, None),
    ("PD14", "GPIO_Analog", "LCD_D0", "analog", None, None),
    ("PD15", "GPIO_Analog", "LCD_D1", "analog", None, None),
    # GPIOE
    ("PE0", "GPIO_Analog", "T_SCK", "analog", None, None),
    ("PE1", "GPIO_Analog", "EXP_PE1", "analog", None, None),
    ("PE2", "GPIO_Analog", "T_MOSI", "analog", None, None),
    ("PE3", "GPIO_Analog", "T_MISO", "analog", None, None),
    ("PE4", "GPIO_Analog", "T_PEN", "analog", None, None),
    ("PE5", "TIM9_CH1", "TIM9_CH1", "auto", None, None),
    ("PE6", "TIM9_CH2", "TIM9_CH2", "auto", None, None),
    ("PE7", "GPIO_Analog", "LCD_D4", "analog", None, None),
    ("PE8", "GPIO_Analog", "LCD_D5", "analog", None, None),
    ("PE9", "GPIO_Analog", "LCD_D6", "analog", None, None),
    ("PE10", "GPIO_Analog", "LCD_D7", "analog", None, None),
    ("PE11", "GPIO_Analog", "LCD_D8", "analog", None, None),
    ("PE12", "GPIO_Analog", "LCD_D9", "analog", None, None),
    ("PE13", "GPIO_Analog", "LCD_D10", "analog", None, None),
    ("PE14", "GPIO_Analog", "LCD_D11", "analog", None, None),
    ("PE15", "GPIO_Analog", "LCD_D12", "analog", None, None),
    # GPIOH (HSE)
    ("PH0-OSC_IN", "RCC_OSC_IN", "HSE_IN", "auto", None, None),
    ("PH1-OSC_OUT", "RCC_OSC_OUT", "HSE_OUT", "auto", None, None),
]

VPS = [
    ("VP_SYS_VS_Systick", "SYS_VS_Systick", "SysTick"),
    ("VP_TIM3_VS_ClockSourceINT", "TIM3_VS_ClockSourceINT", "Internal"),
    ("VP_TIM1_VS_ClockSourceINT", "TIM1_VS_ClockSourceINT", "Internal"),
    ("VP_TIM2_VS_ClockSourceINT", "TIM2_VS_ClockSourceINT", "Internal"),
    ("VP_TIM4_VS_ClockSourceINT", "TIM4_VS_ClockSourceINT", "Internal"),
    ("VP_TIM5_VS_ClockSourceINT", "TIM5_VS_ClockSourceINT", "Internal"),
    ("VP_TIM8_VS_ClockSourceINT", "TIM8_VS_ClockSourceINT", "Internal"),
    ("VP_TIM9_VS_ClockSourceINT", "TIM9_VS_ClockSourceINT", "Internal"),
    ("VP_USB_DEVICE_VS_USB_DEVICE_CDC_FS", "USB_DEVICE_VS_USB_DEVICE_CDC_FS", "CDC_FS"),
]

order = []
for pin, sig, label, kind, params, pval in PINS:
    order.append(pin)
    K[f"{pin}.Signal"] = sig
    K[f"{pin}.Locked"] = "true"
    if kind == "auto":
        continue
    if kind == "out":
        K[f"{pin}.GPIOParameters"] = "GPIO_Speed,PinState,GPIO_Label"
        K[f"{pin}.GPIO_Speed"] = "GPIO_SPEED_FREQ_LOW"
        K[f"{pin}.PinState"] = f"GPIO_PIN_{pval}"
    elif kind == "in":
        K[f"{pin}.GPIOParameters"] = params
        K[f"{pin}.GPIO_PuPd"] = pval
    elif kind == "analog":
        K[f"{pin}.GPIOParameters"] = "GPIO_Label"
    if label:
        K[f"{pin}.GPIO_Label"] = label
for vp, sig, mode in VPS:
    order.append(vp)
    K[f"{vp}.Signal"] = sig
    K[f"{vp}.Mode"] = mode

for n, p in enumerate(order):
    K[f"Mcu.Pin{n}"] = p
K["Mcu.PinsNb"] = str(len(order))

lines = [f"{k}={v}" for k, v in K.items()]
lines.sort(key=lambda l: l.split("=", 1)[0].encode())
data = ("#MicroXplorer Configuration settings - do not modify\r\n"
        + "\r\n".join(lines) + "\r\nboard=custom\r\n").encode("ascii")
SRC.write_bytes(data)
print("final merge written:", len(data), "bytes | pins:", len(order),
      "| IPs:", len(IPS))
