"""Run the production startup functions with fake peripherals and injected faults.

This isolates startup ordering/fault paths; it does not emulate MCU interrupts
or test electrical CAN/SPI behavior. The functions are extracted unchanged from
the source so no second copy of startup logic is maintained by the test.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'Application/Setup/Src/main_cpp.cpp').read_text()


def function(signature):
    start = source.index(signature)
    pos = source.index('{', start)
    depth = 1
    end = pos + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


body = function('extern "C" void main_cpp(void)')
fault = function('[[noreturn]] void startup_fault(uint8_t flags)')
# Symbol values do not affect startup ordering. Flash must point to host memory.
tokens = sorted(set(re.findall(r'\b(?:HAL_TIM_PERIOD_ELAPSED_CB_ID|HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID|ADC_[A-Z_]+|HRTIM_[A-Z0-9_]+|SLEW_200V|DLY_TARGET_1P2US)\b', body)))
constants = '\n'.join(f'constexpr int {name} = {i + 1};' for i, name in enumerate(tokens))
methods = sorted(set(re.findall(r'drv8316\.(\w+)\(', body)))
driver = 'struct Driver {\n' + '\n'.join(f'template<class... A> void {name}(A...) {{}}' for name in methods) + '\n} drv8316;\n'
noops = ['HAL_ADCEx_Calibration_Start', 'HAL_ADC_RegisterCallback',
         'HAL_ADCEx_InjectedStart', 'HAL_ADCEx_InjectedStart_IT',
         'HAL_ADC_Start_DMA', '__HAL_ADC_ENABLE_IT', '__HAL_ADC_DISABLE_IT',
         'HAL_TIM_Base_Start', 'HAL_HRTIM_WaveformCounterStart_IT',
         'HAL_HRTIM_WaveformCounterStart']
stubs = '\n'.join(f'template<class... A> int {name}(A...) {{return 0;}}' for name in noops)
harness = r'''
#include <cassert>
#include <cstdint>
#include <iostream>
struct Done {};
constexpr int HAL_OK=0, HAL_ERROR=1, GPIO_PIN_RESET=0, MOTOR_POLE_PAIRS=11;
constexpr uint8_t ERROR_SPI=2, ERROR_CAN=1, ERROR_HAL_TIM=32, ERROR_HAL_HRTIM=64;
uint32_t tick=0, FLASH_UPDATE=0, flash_marker=0, slave_boot_stage=0;
const uintptr_t ENCODER_FLASH_UPDATE=reinterpret_cast<uintptr_t>(&flash_marker);
struct Position { float p=0, direction=0; int rev=0; };
struct State {
  Position position;
  struct { bool can_error=false; } error_flags;
  int CONTROL_MODE=0;
} state;
int hadc1, hadc2, hadc3, hadc5, hhrtim1, htim2, htim6, htim7, htim16, htim17;
uint16_t s_adc5_buf[2];
struct { uint32_t BSRR=0; } gpio;
auto *DRVOFF_GPIO_Port=&gpio;
constexpr uint32_t DRVOFF_Pin=128;
int scenario=0, can_starts=0, status_ticks=0;
bool status_timer=false, gains_ready=false, pwm=false, calibrated=false;
uint8_t error_flags=0;
uint8_t Error_IsActive() { return error_flags; }
uint8_t GET_ERROR_FLAGS(State*) { return error_flags; }
void Error_Raise(uint8_t flags) { error_flags |= flags; pwm=false; }
void _Error_Handler() { pwm=false; }
uint32_t HAL_GetTick() { return tick; }
void TIM17_PeriodElapsedCB(int*) { ++status_ticks; }
void HAL_Delay(uint32_t ms) {
  tick+=ms;
  if(status_timer) TIM17_PeriodElapsedCB(&htim17);
  if(slave_boot_stage==255 || slave_boot_stage==5) throw Done{};
}
void HAL_GPIO_WritePin(decltype(DRVOFF_GPIO_Port), uint32_t, int) { gpio.BSRR=0; }
void load_control_params_from_flash() {}
void current_loop_init() { gains_ready=true; }
void velocity_loop_init() {}
void position_loop_init() {}
void load_cogging_lut_from_flash() {}
namespace CAN_Handler {
void FDCAN1_SetupFiltersAndStart() {
  ++can_starts;
  assert(gains_ready && state.position.p==MOTOR_POLE_PAIRS);
  if(scenario==4) { state.error_flags.can_error=true; Error_Raise(ERROR_CAN); }
}
}
struct Encoder {
  int start() {
    assert(can_starts==1 && status_timer);
    return scenario==1 ? HAL_ERROR : HAL_OK;
  }
  int readAngle(uint16_t &word) { word=0; return scenario==2 ? HAL_ERROR : HAL_OK; }
} encoder;
void initializePosition(Position*, int, Encoder&) {
  calibrated=true;
  if(scenario==3) Error_Raise(ERROR_SPI);
}
void TIM6_PeriodElapsedCB(int*) { if(scenario==5) Error_Raise(ERROR_SPI); }
void TIM7_PeriodElapsedCB(int*) {}
void TIM16_PeriodElapsedCB(int*) {}
void HAL_ADC_ConversionENDCallback(int*) {}
int HAL_TIM_RegisterCallback(int*, int, void(*)(int*)) { return HAL_OK; }
int HAL_TIM_Base_Start_IT(int *timer) {
  if(timer==&htim17) status_timer=true;
  return HAL_OK;
}
template<class... A> int HAL_HRTIM_WaveformOutputStart(A...) { pwm=true; return HAL_OK; }
'''
checks = r'''
int main() {
  for(scenario=0; scenario<=5; ++scenario) {
    tick=0; slave_boot_stage=0; error_flags=0; state={};
    status_timer=false; gains_ready=false; pwm=false; calibrated=false;
    can_starts=0; status_ticks=0; gpio.BSRR=0;
    try { main_cpp(); } catch(const Done&) {}
    assert(can_starts==1);
    if(scenario==0) {
      assert(slave_boot_stage==5 && pwm && status_ticks>0 && calibrated);
    } else {
      assert(slave_boot_stage==255 && !pwm && gpio.BSRR==DRVOFF_Pin);
      if(scenario==4) assert(error_flags==ERROR_CAN);
      else assert(status_timer && status_ticks>0 && error_flags==ERROR_SPI);
      if(scenario==1 || scenario==2) assert(!calibrated);
    }
  }
  std::cout << "Startup tests passed: normal, receiver init, no frames, calibration, CAN init, position seed\n";
}
'''
with tempfile.TemporaryDirectory(prefix='slave-startup-test-') as folder:
    cpp = Path(folder) / 'startup_test.cpp'
    exe = Path(folder) / 'startup_test'
    cpp.write_text(harness + constants + '\n' + driver + stubs + '\n' + fault + '\n' + body + '\n' + checks)
    subprocess.run(['g++', '-std=c++20', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
