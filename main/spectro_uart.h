#pragma once

// Initialise UART2 and start the H1 spectrometer task.
// No-op unless CONFIG_SPECTRUM_ENABLE_SPECTRO is set.
void spectro_start(void);
