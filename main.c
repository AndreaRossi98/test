#include <stdbool.h>        //%%%%%%%%%%%%%%____MIA PROVA___%%%%%%%%
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "bsp.h"
#include "hardfault.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ant.h"
#include "nrf_pwr_mgmt.h"
#include "ant_interface.h"
#include "ant_parameters.h"
#include "ant_channel_config.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"
#include "nrf_gpio.h" //potrebbe non servire
#include "nrf_drv_saadc.h"
//#include "nrf_drv_clock.h" //da qualche errore, rimosso il .c (da lucchesini posso comunque commentarlo e non da errore)
#include "nrf_nvmc.h"
#include "app_timer.h"

#include "nrf_drv_twi.h"
//includere tutte le librerie per i sensori
#include "sps30.h"
#include "scd4x_i2c.h"
#include "bme280.h"
#include "lis3dh_acc_driver.h"
#include <math.h>
//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% CONSTANT DEFINITION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
#define TWI_ADDRESSES   127         //Number of possible TWI addresses.
#define APP_ANT_OBSERVER_PRIO   1   // Application's ANT observer priority. You shouldn't need to modify this value.
#define SAADC_BATTERY   0           //Canale tensione della batteria
#define TIMEOUT_VALUE   1000   //1000       // 25 mseconds timer time-out value. Interrupt a 40Hz
#define START_ADDR  0x00011200      //indirizzo di partenza per salvataggio dati in memoria non volatile
#define LED             07

#define NO2_CHANNEL     0           //NO2 channel for ADC
#define CO_CHANNEL      2           //CO channel for ADC

#define _2_SEC          2       //interval of 2 seconds
#define _20_SEC         20      //interval of 20 seconds


//=============================================================================================================================================================================

struct data{
    uint8_t giorno;
    uint8_t mese;
    uint8_t anno;
};

struct val_campionati{
    float Temp;
    float Hum;
    float Pres;
    float PM1p0;    //controlla che sia corretto, scegliere quali valori guardare di PM
    float PM2p5;
    float CO2;
    float CO;
    float NO2;
    float VOC;
};

struct mics6814_data{
    uint16_t NO2;
    uint16_t CO;
};

struct scd4x_data{
    uint16_t CO2;
    int32_t Temperature;
    int32_t Humidity;
};
//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% GLOBAL VARIABLE DEFINITION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
volatile int count=0, stato=0, i=0;
const float deltat = 0.025;
int notshown = 1;           // per il log

int rtc_count = 0;
uint8_t flag_misurazioni = 0;
float partial_calc = 0;        //variable to maintein partial calculation

nrf_saadc_value_t adc_val;  //variabile per campionamento 
ret_code_t err_code;        //variabile per valore di ritorno

//PUò ESSERE CHE QUESTE DUE VARIABILI NON MI SERVANO POI (al momento serve per ant)
volatile int cal_rec = 0;   //impedisce che si faccia più di una calibrazione, se se ne vuole fare un'altra bisogna spegnere e riaccendere
volatile int flag_cal = 0;  //flag che definisce calibrazione in corso

//VARIABILI DEI SENSORI
struct bme280_dev           dev_bme280;         //struct for BME280
struct bme280_data          measure_bme280;     //struct for measured values by BME280
struct sps30_measurement    measure_sps30;      //struct for measured values by SPS30
struct lis3dh_data          measure_lis3dh;     //struct for measured values by LIS3DH
struct mics6814_data        measure_mics6814;   //struct for measured values by MICS6814
struct scd4x_data           measure_scd4x;     //struct for measured values by SCD41
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% TWI COMMUNICATION %%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
//TWI instance ID.
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

//TWI instance
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//TWI initialization
void twi_init (void)
{
    ret_code_t err_code;
    const nrf_drv_twi_config_t twi_config = {
       .scl                = 19,
       .sda                = 18,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}
//=============================================================================================================================================================================

APP_TIMER_DEF(m_repeated_timer_id);     /*Handler for repeated timer */

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% SAADC %%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)  //non serve
{
}

void saadc_init(void)   //prova a mettere low power mode
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel1_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    nrf_saadc_channel_config_t channel3_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(NO2_CHANNEL, &channel1_config);
    APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_saadc_channel_init(CO_CHANNEL, &channel3_config);
    APP_ERROR_CHECK(err_code); 
}
//Transform the ADC value in bit in voltage value
float adc_to_volts (int adc)
{
    float volts = (adc + 5) * 3.6 / 1023;
    return volts;
}
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void utils_setup(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_init(BSP_INIT_LEDS,
                        NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}
//=============================================================================================================================================================================

/*
*   Si può aggiungere funzione per lettura e scrittura
*   in memoria non volatile nrf_nvmc
*/

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% ANT COMMUNICATION %%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/
void ant_send(int campione, int counter, int quat1, int quat2, int quat3, int quat4)
{
    uint8_t message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE];

    memset(message_payload, 0, ANT_STANDARD_DATA_PAYLOAD_SIZE);
    // Assign a new value to the broadcast data.
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 8] = 4; //DEVICENUMBER
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 7] = campione; 
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 6] = 0;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 5] = counter;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 4] = quat1;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 3] = quat2;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 2] = quat3;
    message_payload[ANT_STANDARD_DATA_PAYLOAD_SIZE - 1] = quat4;

    // Broadcast the data.
    ret_code_t err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,
                                                    ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                                    message_payload);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Messaggio numero %d, ret code ant broadcast: %d", counter, err_code);
}

//Function for handling stack event
void ant_evt_handler(ant_evt_t * p_ant_evt, void * p_context)
{
    if (p_ant_evt->channel == BROADCAST_CHANNEL_NUMBER && flag_cal == 0)  //durante la calibrazione ignora tutti i messaggi che arrivano dal master
    {
        switch (p_ant_evt->event)
        {
            case EVENT_RX:
                if (p_ant_evt->message.ANT_MESSAGE_ucMesgID == MESG_BROADCAST_DATA_ID)
                {
printf("\nRicevuto: ");
for(int i = 0;i<8;i++)
printf("%d", p_ant_evt->message.ANT_MESSAGE_aucPayload [i]);
printf("\n");
//if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00]) printf("Connesso");                    
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x80 )   //se il primo byte del payload è zero e l'ultimo è 128
                    { 									
                        stato=0;	//ferma l'acquisizione												
                        nrf_gpio_pin_set(LED);
                        sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
                        NRF_LOG_INFO("Ricevuto messaggio di stop acquisizione");
                          
                    }
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0x00)   //se il primo byte del payload è zero avvia l'acquisizione
                    {
                        NRF_LOG_INFO("Inizio acquisizione");	
                        sd_ant_pending_transmit_clear (BROADCAST_CHANNEL_NUMBER, NULL); //svuota il buffer, utile per una seconda acquisizione
                        nrf_gpio_pin_clear(LED);
                        count=0;
                        i=0;
                        stato = 1;
                    }   
                    if (p_ant_evt->message.ANT_MESSAGE_aucPayload [0x00] == 0x00 && p_ant_evt->message.ANT_MESSAGE_aucPayload [0x07] == 0xFF && cal_rec == 0)   
                    {       //messaggio di inizio calibrazione
                        cal_rec = 1;
                        stato = 0;
                        //calibrazione();
                        //salva_calib_flash();
                    }		
                }
                break;
            case EVENT_RX_SEARCH_TIMEOUT:	   //in caso di timeout riapre il canale
                err_code = sd_ant_channel_open(BROADCAST_CHANNEL_NUMBER);
                APP_ERROR_CHECK(err_code); 
                break;							

            default:
                break;
        }
    }
}

NRF_SDH_ANT_OBSERVER(m_ant_observer, APP_ANT_OBSERVER_PRIO, ant_evt_handler, NULL);

//Function for ANT stack initialization.
static void softdevice_setup(void)
{
    ret_code_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);
}

// Function for setting up the ANT module to be ready for RX broadcast.
static void ant_channel_rx_broadcast_setup(void)
{
    ant_channel_config_t broadcast_channel_config =
    {
        .channel_number    = BROADCAST_CHANNEL_NUMBER,
        .channel_type      = CHANNEL_TYPE_SLAVE,
        .ext_assign        = 0x00,
        .rf_freq           = RF_FREQ,
        .transmission_type = CHAN_ID_TRANS_TYPE,
        .device_type       = CHAN_ID_DEV_TYPE,
        .device_number     = CHAN_ID_DEV_NUM,
        .channel_period    = CHAN_PERIOD,
        .network_number    = ANT_NETWORK_NUM,
    };

    ret_code_t err_code = ant_channel_init(&broadcast_channel_config);
    APP_ERROR_CHECK(err_code);

    // Open channel.
    err_code = sd_ant_channel_open(BROADCAST_CHANNEL_NUMBER);
    APP_ERROR_CHECK(err_code);
}
//=============================================================================================================================================================================

//=============================================================================================================================================================================
/*
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%% TIMER HANDLER %%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

static void repeated_timer_handler(void * p_context)  //app timer, faccio scattare ogni un secondo
{ 
    rtc_count ++;
    //controllo della batteria, ogni quanto? come il campionamento, e come fare controllo? voltage divider?
    //err_code = nrf_drv_saadc_sample_convert(SAADC_BATTERY, &sample);   //lettura ADC
    APP_ERROR_CHECK(err_code);

    //1 sec
    //Lettura dati VOC
    
    //2 sec
    if ((rtc_count % _2_SEC) == 0)
    {
        //ant_send(1, 1, 11, 11, 11, 11);
        //invio ant
        printf("\n2 sec\n");
        nrf_gpio_pin_toggle(LED);
    }

    //20 sec
    if ((rtc_count % _20_SEC) == 0)
    {
        //campiono tutti i valori, flag e si fa nel main, confronto con umidità
        //rtc_count = 0 se non mi serve intervallo più grande
        flag_misurazioni = 1; //eseguire misurazioni ogni 20 sec nel main
    }
    
    //1 ora per sgp30 baseline iaq (capire se serve)                                                                                     
}
//=============================================================================================================================================================================


int main(void)
{
printf("inizio\n");
    nrf_gpio_cfg_output(LED);
    nrf_gpio_pin_set(LED);
    //Inizializzazione di tutte le componenti
    log_init();
    saadc_init();
    twi_init();

    softdevice_setup();
    ant_channel_rx_broadcast_setup();
    utils_setup();
printf("Sono qua\n");
    sd_ant_channel_radio_tx_power_set(BROADCAST_CHANNEL_NUMBER, RADIO_TX_POWER_LVL_4, NULL); 	//potenza trasmissione
    
    uint8_t  message_addr[ANT_STANDARD_DATA_PAYLOAD_SIZE];
    memset(message_addr, 4, ANT_STANDARD_DATA_PAYLOAD_SIZE); //DEVICENUMBER
for(i=0;i<5;i++){		
    err_code = sd_ant_broadcast_message_tx(BROADCAST_CHANNEL_NUMBER,         //invia messaggio di accensione
                                           ANT_STANDARD_DATA_PAYLOAD_SIZE,
                                           message_addr);
nrf_delay_ms(500);}
printf("Invio dato: ");
for(i=0;i<8;i++)
{
printf("%d", message_addr[i]);
}
printf("\n\n");
    app_timer_init();
    err_code = app_timer_create(&m_repeated_timer_id,
                            APP_TIMER_MODE_REPEATED,
                            repeated_timer_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(LED);
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
printf("Timer\n");    
    err_code = app_timer_start(m_repeated_timer_id, APP_TIMER_TICKS(TIMEOUT_VALUE), NULL);
    APP_ERROR_CHECK(err_code);	
/*
    sps30_init();
    //scd41 non serve init
    bme280_init_set(&dev_bme280);
    lis3dh_init();
*/
    /* 
     * Inizializzazione dei sensori e settaggio dei parametri con le funzioni apposite
     * Tenere traccia del giorno e vedere quanto è passato per fare pulizia della ventola
     * 
     */

    // Main loop.
    for (;;)
    {
/*        if(flag_misurazioni == 1)   //esegui tutte le misurazioni tranne VOC
        {
            float partial_calc = 0;        //variable to maintein partial calculation
//ha senso guardare se restituiscono o meno errore queste funzioni?
            
            //unire sps e scd per fare un unico delay, o separarli per consumo di corrente massimo disponibile
            
            //SPS30 
            sps30_wake_up();
            sps30_start_measurement();
            nrf_delay_ms(1000);
            sps30_read_measurement(&measure_sps30);
            sps30_stop_measurement();
            sps30_sleep();

            //SCD41
            scd4x_wake_up();
            scd4x_measure_single_shot();
            nrf_delay_ms(100);
            scd4x_read_measurement(&measure_scd4x.CO2, &measure_scd4x.Temperature, &measure_scd4x.Humidity);
            scd4x_power_down();

            //MICS6814
            nrfx_saadc_sample_convert(NO2_CHANNEL, &adc_val); //A1
            partial_calc = (5 - adc_to_volts(adc_val))/adc_to_volts(adc_val);
            measure_mics6814.NO2 = pow(10, (log10(partial_calc) -0.804)/(1.026))*1000;
            nrfx_saadc_sample_convert(CO_CHANNEL, &adc_val); //A3
            partial_calc = (5 - adc_to_volts(adc_val))/adc_to_volts(adc_val);
            measure_mics6814.CO = pow(10, (log10(partial_calc)-0.55)/(-0.85));
            
            //BME280
            bme280_set_sensor_mode(BME280_FORCED_MODE, &dev_bme280);
            bme280_get_sensor_data(BME280_ALL, &measure_bme280, &dev_bme280);
    
            //LIS3DH
            //scegliere ogni quanto campionare         
        }
*/
        //NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
        __WFI();//GO INTO LOW POWER MODE
    }
}


/**
 *@}
 **/
