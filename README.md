# i.MX 8M Mini Heterogenous Low Power Voice Control Solution


This is the materials used in the application note: i.MX 8M Mini Heterogenous Low Power Voice Control Solution. This application note aims at showing how to deploy low power local voice on a NXP SoC. This application note demonstrates how to integrate this technology on NXP products while minimizing power consumption. More precisely, by leveraging i.MX  heterogenous computing architecture, the following use case enables to continuously spot for wakeword when the main system is set in deep sleep mode (DSM), power consumption constraints of home appliances. The solution presented in the current document targets i.MX 8M Mini platforms using both Cortex-A53 and Cortex-M4. The solution can be applied to other i.MX platforms but will need to be adapted depending on which software component manages the power states and handles access to resources shared between Cortex-A53 and Cortex-M4. For this platform, power management and resource access are managed in the Arm Trusted Firmware (ATF), Linux BSP and Cortex-M4 app.

## Version

The M4 app "wake_word_low_power_demo" was created on the MCUxpresso SDK_2.8.0. <br/>
The kernel and atf patches work on Linux 5.4.47_2.2.0

## Link

For further information on this demo please check the Application Note document "i.MX 8M Mini Heterogenous Low Power Voice Control Solution" available on nxp.com.

## License

License BSD-3-Clause: wake_word_low_power_demo, atf_wake_word_low_power_demo.patch <br/>
License GPL-2.0: kernel_wake_word_low_power_demo.patch

