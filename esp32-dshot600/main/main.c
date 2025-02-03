#include "driver/rmt.h"
#include "esp_log.h"

// 硬件配置
#define DSHOT_GPIO      18      // 连接电调信号线
#define RMT_CHANNEL     RMT_CHANNEL_0

// 时钟配置（关键参数）
#define APB_CLK_FREQ    80      // ESP32 APB时钟实际为80MHz
#define RMT_CLK_DIV     4       // 80MHz / 4 = 20MHz → 每tick 50ns
#define RMT_TICK_NS     50      // 每个RMT tick的纳秒数

// DShot600时序参数（计算到RMT ticks）
#define T0H_TICKS   (417 / RMT_TICK_NS)   // 417ns → 8 ticks (400ns)
#define T0L_TICKS   (1250 / RMT_TICK_NS)  // 1250ns → 25 ticks (1250ns)
#define T1H_TICKS   T0L_TICKS             // 1的高电平时间
#define T1L_TICKS   T0H_TICKS             // 1的低电平时间
#define SYNC_TICKS  (2000 / RMT_TICK_NS)  // 同步间隔2µs → 40 ticks

// 数据包结构
typedef struct {
    uint16_t throttle : 11;  // 油门值 0-2047
    uint8_t telemetry : 1;   // 遥测请求
    uint8_t crc : 4;         // CRC校验
} dshot_packet_t;

// RMT初始化（单电调专用）
void dshot_init() {
    rmt_config_t config = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_CHANNEL,
        .gpio_num = DSHOT_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .tx_config = {
            .carrier_freq_hz = 0,   // 禁用载波
            .loop_en = false,
            .idle_level = RMT_IDLE_LEVEL_LOW,
            .idle_output_en = true,
        }
    };
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

// CRC计算（多项式: 0b10011）
static uint8_t calculate_crc(uint16_t data) {
    uint16_t crc = data ^ (data >> 4) ^ (data >> 8);
    return crc & 0x0F;
}

// 生成RMT数据项（优化内存布局）
static void generate_rmt_items(uint16_t value, rmt_item32_t* items) {
    // 构造数据包：移位处理使高位先发送
    dshot_packet_t packet = {
        .throttle = value,
        .telemetry = 0,  // 默认不请求遥测
        .crc = calculate_crc((value << 1) | 0) // 计算CRC
    };
    
    uint16_t raw_data = *(uint16_t*)&packet;
    
    // 生成每个位的RMT项（从最高位开始）
    for(int i=15; i>=0; i--) {
        bool bit = (raw_data >> i) & 0x1;
        items[15-i] = (rmt_item32_t) {
            .duration0 = bit ? T1H_TICKS : T0H_TICKS,
            .level0 = 0,
            .duration1 = bit ? T1L_TICKS : T0L_TICKS,
            .level1 = 1,
        };
    }
    
    // 添加同步间隔（低电平）
    items[16] = (rmt_item32_t) {
        .duration0 = SYNC_TICKS,
        .level0 = 0,
        .duration1 = 0,
        .level1 = 0
    };
}

// 发送DShot指令（带互斥锁保证时序）
void dshot_send_command(uint16_t throttle, bool telemetry) {
    static StaticSemaphore_t mutex_buffer;
    static SemaphoreHandle_t rmt_mutex = NULL;
    static rmt_item32_t rmt_items[17]; // 静态分配内存
    
    // 首次调用时初始化互斥锁
    if(rmt_mutex == NULL) {
        rmt_mutex = xSemaphoreCreateMutexStatic(&mutex_buffer);
    }
    
    // 限制油门范围（DShot特殊指令处理）
    if(throttle > 2047) throttle = 2047;
    
    // 构造数据包（原子操作保护）
    xSemaphoreTake(rmt_mutex, portMAX_DELAY);
    generate_rmt_items((throttle & 0x07FF) | (telemetry << 11), rmt_items);
    
    // 发送并等待完成
    ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, rmt_items, 17, true));
    ESP_ERROR_CHECK(rmt_wait_tx_done(RMT_CHANNEL, pdMS_TO_TICKS(10)));
    xSemaphoreGive(rmt_mutex);
}

/* 使用示例 */
void app_main() {
    dshot_init();
    
    // 上电后持续发送100ms低信号（部分电调需要初始化）
    for(int i=0; i<100; i++) {
        dshot_send_command(0, false);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // 正常操作示例
    while(1) {
        // 满油门带遥测请求
        dshot_send_command(2047, true);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        // 停转指令
        dshot_send_command(0, false);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
