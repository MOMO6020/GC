.
├─.vscode                   # VSCode工程配置目录（编译参数、调试配置等）
├─GC                        # 核心工程目录（STM32F4硬件层+算法层核心代码）
│  ├─MDK-ARM                # Keil MDK-ARM工程目录
│  │  ├─.vscode             # Keil工程关联的VSCode配置子目录
│  │  │  ├─c_cpp_properties.json # C/C++语法解析、编译路径配置
│  │  │  ├─keil-assistant.log     # Keil助手插件日志
│  │  │  ├─uv4.log                # Keil UV4编译器日志
│  │  │  └─uv4.log.lock           # Keil日志文件锁
│  │  └─eMPL                 # 运动处理库目录（MPU6050 DMP驱动）
│  │     └─inv_mpu_dmp_motion_driver.c # MPU6050 DMP轻敲阈值配置等运动驱动
│  ├─Drivers                # STM32F4xx硬件驱动层（CubeMX生成/官方HAL库）
│  │  ├─CMSIS               # CMSIS标准库目录
│  │  │  └─DSP_Lib          # DSP数字信号处理库
│  │  │     └─Source        # DSP算法源码
│  │  │        └─TransformFunctions # 变换函数模块
│  │  │           └─arm_cfft_radix4_q15.c # 基4FFT蝶形运算（q15定点数/SIMD加速）
│  │  └─STM32F4xx_HAL_Driver # STM32F4 HAL库核心
│  │     ├─Src              # HAL库源文件
│  │     │  ├─stm32f4xx_hal_eth.c    # 以太网MAC/DMA配置驱动
│  │     │  ├─stm32f4xx_hal_rcc_ex.c # RCC扩展时钟配置（PLLI2S/PLLSAI/CLK48等）
│  │     │  ├─stm32f4xx_ll_fmc.c     # FMC外设LL库驱动
│  │     │  └─stm32f4xx_hal_adc_ex.c # ADC扩展功能驱动（DMA错误处理等）
│  ├─Inc                    # 工程头文件目录（CubeMX自动生成，通常无需修改）
│  └─Src                    # 工程源文件目录（CubeMX自动生成，通常无需修改）
├─app                       # 用户应用层代码（主函数、任务创建等顶层逻辑）
├─bsp                       # 板级支持包（BSP）
│  ├─can                    # CAN总线驱动与初始化代码
│  ├─log                    # 日志系统模块（调试输出）
│  ├─structure              # 通用数据结构定义
│  ├─system                 # 系统工具模块（DWT计时、互斥锁、时间管理等）
│  ├─uart                   # UART串口通信驱动
│  └─usb                    # USB虚拟串口驱动（存在问题 TODO）
├─modules                   # 功能模块目录（业务层封装，各子系统模块）
│  ├─buzzer                 # 蜂鸣器控制模块（未实现）
│  ├─controller             # 控制器模块（遥控器、手柄等输入解析）
│  ├─daemon                 # 守护进程模块（监控系统运行状态）
│  ├─imu                    # 惯性测量单元模块（姿态角度传感器驱动）
│  ├─motor                  # 电机模块（后续考虑抽象不同品牌电机接口）
│  │  └─DJIMotor            # DJI电机专用驱动封装
│  ├─pid                    # PID控制模块
│  ├─umt                    # 线程间通信模块（移植自上交开源，后续考虑重新实现）
│  └─vision                 # 视觉模块（处理上位机回传数据，未经测试 TODO）
├─examples                  # 示例代码、测试用例
│  └─Serial_motor           # 电机串口调试示例
├─Middlewares               # 第三方中间件库
│  ├─ST                     # ST官方中间件（HAL库、USB库等）
│  └─Third_Party            # 第三方组件
│     ├─FreeRTOS            # FreeRTOS实时操作系统源码
│     └─SEGGER              # SEGGER RTT/RTT Viewer支持库
├─cmake                     # CMake相关脚本
│  └─stm32cubemx            # STM32CubeMX生成的CMake配置文件目录
├─startup_stm32f407xx.s     # 启动汇编文件（STM32启动时执行）
├─STM32F407.svd             # SVD文件（调试器显示寄存器信息）
├─STM32F407XX_FLASH.ld      # 链接脚本（定义Flash与RAM地址分布）
├─openocd_dap.cfg           # OpenOCD调试配置（DAPLink使用）
├─openocd_stlink.cfg        # OpenOCD调试配置（ST-Link使用）
├─CMakeLists.txt            # 顶层CMake构建脚本
├─README.md                 # 工程说明文档
├─LICENSE                   # 开源许可文件
├─.gitignore                # Git忽略规则配置文件
└─.gitattributes            # Git属性配置（换行符、差异合并策略等）
