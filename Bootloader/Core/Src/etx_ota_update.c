/*
 * etx_ota_update.c
 *
 * Author: EmbeTronicX (Fixed Rollback Logic)
 */

#include <stdio.h>
#include "etx_ota_update.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

/* Buffer to hold the received data */
static uint8_t Rx_Buffer[ ETX_OTA_PACKET_MAX_SIZE ];

/* OTA State */
static ETX_OTA_STATE_ ota_state = ETX_OTA_STATE_IDLE;

/* Firmware info */
static uint32_t ota_fw_total_size;
static uint32_t ota_fw_crc;
static uint32_t ota_fw_received_size;
static uint8_t  slot_num_to_write;

/* Configuration */
ETX_GNRL_CFG_ *cfg_flash = (ETX_GNRL_CFG_*) (ETX_CONFIG_FLASH_ADDR);

/* Retry Counter */
static uint8_t ota_retry_count = 0;
#define MAX_OTA_RETRIES 5

/* Hardware Handles */
extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;

/* Prototypes */
static uint16_t etx_receive_chunk( uint8_t *buf, uint16_t max_len );
static ETX_OTA_EX_ etx_process_data( uint8_t *buf, uint16_t len );
static void etx_ota_send_resp( uint8_t type );
static HAL_StatusTypeDef write_data_to_slot( uint8_t slot_num, uint8_t *data, uint16_t data_len, bool is_first_block );
static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data, uint32_t data_len );
static uint8_t get_available_slot_number( void );
static HAL_StatusTypeDef write_cfg_to_flash( ETX_GNRL_CFG_ *cfg );
static uint32_t calculate_crc32(const uint8_t *data, uint32_t length);

//Hàm tính CRC32

static uint32_t calculate_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04C11DB7;
    for (uint32_t i = 0; i < length; i++) {
        crc ^= (uint32_t)data[i] << 24;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80000000) crc = (crc << 1) ^ poly;
            else crc = (crc << 1);
        }
    }
    return crc;
}

ETX_OTA_EX_ etx_ota_download_and_flash( void )
{
  ETX_OTA_EX_ ret  = ETX_OTA_EX_OK;
  uint16_t    len;

  if(ota_retry_count > 0)
      printf("Waiting for OTA data... (Retry %d/%d)\r\n", ota_retry_count, MAX_OTA_RETRIES);
  else
      printf("Waiting for OTA data...\r\n");

  /* Reset variables */
  ota_fw_total_size    = 0u;
  ota_fw_received_size = 0u;
  ota_fw_crc           = 0u;
  ota_state            = ETX_OTA_STATE_START;
  slot_num_to_write    = 0xFFu;

  do
  {
    memset( Rx_Buffer, 0, ETX_OTA_PACKET_MAX_SIZE );

    // Nhận dữ liệu (Timeout 5s)
    len = etx_receive_chunk( Rx_Buffer, ETX_OTA_PACKET_MAX_SIZE );

    if( len != 0u )
    {
      ret = etx_process_data( Rx_Buffer, len );
    }
    else
    {
      ret = ETX_OTA_EX_ERR;
    }

    // --- XỬ LÝ KẾT QUẢ ---
    if( ret != ETX_OTA_EX_OK )
    {
      printf("Sending NACK\r\n");
      etx_ota_send_resp( ETX_OTA_NACK );

      ota_retry_count++;

      // Nếu lỗi quá 5 lần -> Hủy bỏ OTA và Quay về App cũ
      if (ota_retry_count >= MAX_OTA_RETRIES)
      {
          printf("Too many errors/timeouts (%d). Aborting OTA...\r\n", MAX_OTA_RETRIES);

          //  Xóa cờ OTA REQUEST trong Flash trước khi Reset
          ETX_GNRL_CFG_ cfg;
          memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );

          if(cfg.reboot_cause != ETX_NORMAL_BOOT)
          {
              printf("Clearing OTA Flag in Flash...\r\n");
              cfg.reboot_cause = ETX_NORMAL_BOOT;
              write_cfg_to_flash( &cfg );
          }

          printf("System Resetting to Rollback/Normal Boot...\r\n");
          HAL_Delay(100);
          HAL_NVIC_SystemReset();
      }

      break;
    }
    else
    {
      ota_retry_count = 0;
      etx_ota_send_resp( ETX_OTA_ACK );
    }

  } while( ota_state != ETX_OTA_STATE_IDLE );

  return ret;
}

static ETX_OTA_EX_ etx_process_data( uint8_t *buf, uint16_t len )
{
  ETX_OTA_EX_ ret = ETX_OTA_EX_ERR;

  do
  {
    if( ( buf == NULL ) || ( len == 0u) ) break;

    ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*)buf;
    if( cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD )
    {
      if( cmd->cmd == ETX_OTA_CMD_ABORT ) break;
    }

    switch( ota_state )
    {
      case ETX_OTA_STATE_IDLE:
        printf("ETX_OTA_STATE_IDLE...\r\n");
        ret = ETX_OTA_EX_OK;
        break;

      case ETX_OTA_STATE_START:
        if( cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD && cmd->cmd == ETX_OTA_CMD_START )
        {
          printf("Received OTA START Command\r\n");
          ota_state = ETX_OTA_STATE_HEADER;
          ret = ETX_OTA_EX_OK;
        }
        break;

      case ETX_OTA_STATE_HEADER:
      {
        ETX_OTA_HEADER_ *header = (ETX_OTA_HEADER_*)buf;
        if( header->packet_type == ETX_OTA_PACKET_TYPE_HEADER )
        {
          ota_fw_total_size = header->meta_data.package_size;
          ota_fw_crc        = header->meta_data.package_crc;
          printf("Received OTA Header. FW Size = %ld\r\n", ota_fw_total_size);

          slot_num_to_write = get_available_slot_number();
          if( slot_num_to_write != 0xFF )
          {
            ota_state = ETX_OTA_STATE_DATA;
            ret = ETX_OTA_EX_OK;
          }
        }
      }
      break;

      case ETX_OTA_STATE_DATA:
      {
        ETX_OTA_DATA_ *data = (ETX_OTA_DATA_*)buf;
        uint16_t data_len = data->data_len;
        HAL_StatusTypeDef ex;

        if( data->packet_type == ETX_OTA_PACKET_TYPE_DATA )
        {
          bool is_first_block = false;
          if( ota_fw_received_size == 0 )
          {
            is_first_block = true;

            // Đánh dấu Slot này là KHÔNG HỢP LỆ
            ETX_GNRL_CFG_ cfg;
            memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );
            cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 1u;

            if( write_cfg_to_flash( &cfg ) != HAL_OK ) break;
          }

          ex = write_data_to_slot( slot_num_to_write, buf+4, data_len, is_first_block );

          if( ex == HAL_OK )
          {
            printf("[%ld/%ld]\r\n", ota_fw_received_size/ETX_OTA_DATA_MAX_SIZE, ota_fw_total_size/ETX_OTA_DATA_MAX_SIZE);
            if( ota_fw_received_size >= ota_fw_total_size )
            {
              ota_state = ETX_OTA_STATE_END;
            }
            ret = ETX_OTA_EX_OK;
          }
        }
      }
      break;

      case ETX_OTA_STATE_END:
      {
        ETX_OTA_COMMAND_ *cmd = (ETX_OTA_COMMAND_*)buf;
        if( cmd->packet_type == ETX_OTA_PACKET_TYPE_CMD && cmd->cmd == ETX_OTA_CMD_END )
        {
            printf("Received OTA END. Validating...\r\n");
            uint32_t slot_addr = (slot_num_to_write == 0u) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;

            uint32_t cal_crc = calculate_crc32( (uint8_t*)slot_addr, ota_fw_total_size);
            if( cal_crc != ota_fw_crc )
            {
              printf("ERROR: FW CRC Mismatch\r\n");
              break;
            }
            printf("Checksum OK! Updating Config...\r\n");

            ETX_GNRL_CFG_ cfg;
            memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );

            cfg.slot_table[slot_num_to_write].fw_crc                 = cal_crc;
            cfg.slot_table[slot_num_to_write].fw_size                = ota_fw_total_size;
            cfg.slot_table[slot_num_to_write].is_this_slot_not_valid = 0u; // Valid!
            cfg.slot_table[slot_num_to_write].should_we_run_this_fw  = 1u; // Run next!

            for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
            {
              if( slot_num_to_write != i ) cfg.slot_table[i].should_we_run_this_fw = 0u;
            }

            cfg.reboot_cause = ETX_NORMAL_BOOT;

            if( write_cfg_to_flash( &cfg ) == HAL_OK )
            {
              ota_state = ETX_OTA_STATE_IDLE;
              ret = ETX_OTA_EX_OK;
            }
        }
      }
      break;

      default:
        ret = ETX_OTA_EX_ERR;
        break;
    };
  } while( false );

  return ret;
}

// Hàm nhận dữ liệu với Timeout
static uint16_t etx_receive_chunk( uint8_t *buf, uint16_t max_len )
{
  HAL_StatusTypeDef ret;
  uint16_t index = 0u;
  uint16_t data_len;
  uint32_t cal_data_crc, rec_data_crc;

  // Timeout 5000ms
  const uint32_t RX_TIMEOUT_MS = 5000;

  do
  {
    // 1. Nhận SOF
    ret = HAL_UART_Receive( &huart2, &buf[index], 1, RX_TIMEOUT_MS );
    if( ret != HAL_OK ) break;
    if( buf[index++] != ETX_OTA_SOF ) return 0;

    // 2. Nhận Packet Type
    ret = HAL_UART_Receive( &huart2, &buf[index++], 1, 1000 );
    if( ret != HAL_OK ) break;

    // 3. Nhận Length
    ret = HAL_UART_Receive( &huart2, &buf[index], 2, 1000 );
    if( ret != HAL_OK ) break;
    data_len = *(uint16_t *)&buf[index];
    index += 2u;

    // 4. Nhận Data Payload
    if (data_len > 0) {
        ret = HAL_UART_Receive( &huart2, &buf[index], data_len, 2000 );
        if( ret != HAL_OK ) break;
        index += data_len;
    }

    // 5. Nhận CRC
    ret = HAL_UART_Receive( &huart2, &buf[index], 4, 1000 );
    if( ret != HAL_OK ) break;
    rec_data_crc = *(uint32_t *)&buf[index];
    index += 4u;

    // 6. Nhận EOF
    ret = HAL_UART_Receive( &huart2, &buf[index], 1, 1000 );
    if( ret != HAL_OK ) break;
    if( buf[index++] != ETX_OTA_EOF ) return 0;

    // Verify CRC
    cal_data_crc = calculate_crc32( &buf[4], data_len);
    if( cal_data_crc != rec_data_crc ) {
      printf("CRC Mismatch!\r\n");
      return 0;
    }

  } while( false );

  if( ret != HAL_OK ) index = 0u;
  return index;
}

static void etx_ota_send_resp( uint8_t type )
{
  ETX_OTA_RESP_ rsp = {
    .sof = ETX_OTA_SOF, .packet_type = ETX_OTA_PACKET_TYPE_RESPONSE,
    .data_len = 1u, .status = type, .eof = ETX_OTA_EOF
  };
  rsp.crc = calculate_crc32( (uint8_t*)&rsp.status, 1);
  HAL_UART_Transmit(&huart2, (uint8_t *)&rsp, sizeof(ETX_OTA_RESP_), 1000);
}


static HAL_StatusTypeDef write_data_to_slot( uint8_t slot_num,
                                             uint8_t *data,
                                             uint16_t data_len,
                                             bool is_first_block )
{
  HAL_StatusTypeDef ret;

  do
  {

    if( slot_num >= ETX_NO_OF_SLOTS )
    {
      ret = HAL_ERROR;
      break;
    }

    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //No need to erase every time. Erase only the first time.
    if( is_first_block )
    {
      printf("Erasing the Slot %d Flash memory...\r\n", slot_num);
      //Erase the Flash
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError;

      EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
      if( slot_num == 0 )
      {
        EraseInitStruct.Sector        = FLASH_SECTOR_6;
      }
      else
      {
        EraseInitStruct.Sector        = FLASH_SECTOR_7;
      }
      EraseInitStruct.NbSectors     = 1;                    //erase 1 sectors
      EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

      ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
      if( ret != HAL_OK )
      {
        printf("Flash Erase Error\r\n");
        break;
      }
    }

    uint32_t flash_addr;
    if( slot_num == 0 )
    {
      flash_addr = ETX_APP_SLOT0_FLASH_ADDR;
    }
    else
    {
      flash_addr = ETX_APP_SLOT1_FLASH_ADDR;
    }

    for(int i = 0; i < data_len; i++ )
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
                               (flash_addr + ota_fw_received_size),
                               data[i]
                             );
      if( ret == HAL_OK )
      {
        //update the data count
        ota_fw_received_size += 1;
      }
      else
      {
        printf("Flash Write Error\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while( false );

  return ret;
}

static uint8_t get_available_slot_number( void )
{
  uint8_t   slot_number = 0xFF;

  /* Read the configuration */
  ETX_GNRL_CFG_ cfg;
  memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );

   for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
   {
     if( ( cfg.slot_table[i].is_this_slot_not_valid != 0u ) || ( cfg.slot_table[i].is_this_slot_active == 0u ) )
     {
       slot_number = i;
       printf("Slot %d is available for OTA update\r\n", slot_number);
       break;
     }
   }

   return slot_number;
}

static HAL_StatusTypeDef write_data_to_flash_app( uint8_t *data, uint32_t data_len )
{
  HAL_StatusTypeDef ret;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );

    // clear all flags before you write it to flash
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

    printf("Erasing the App Flash memory...\r\n");
    //Erase the Flash
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector        = FLASH_SECTOR_5;
    EraseInitStruct.NbSectors     = 1;                    //erase 1 sectors 5
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
    if( ret != HAL_OK )
    {
      printf("Flash erase Error\r\n");
      break;
    }

    for( uint32_t i = 0; i < data_len; i++ )
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
                               (ETX_APP_FLASH_ADDR + i),
                               data[i]
                             );
      if( ret != HAL_OK )
      {
        printf("App Flash Write Error\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );

  }while( false );

  return ret;
}

void load_new_app( void )
{
  bool              is_update_available = false;
  uint8_t           slot_num;
  HAL_StatusTypeDef ret;

  /* Read the configuration */
  ETX_GNRL_CFG_ cfg;
  memcpy( &cfg, cfg_flash, sizeof(ETX_GNRL_CFG_) );

  /* Check the slot whether it has a new application. */
   for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
   {
     if( cfg.slot_table[i].should_we_run_this_fw == 1u )
     {
       printf("New Application is available in the slot %d!!!\r\n", i);
       is_update_available               = true;
       slot_num                          = i;

       //update the slot
       cfg.slot_table[i].is_this_slot_active    = 1u;
       cfg.slot_table[i].should_we_run_this_fw  = 0u;

       break;
     }
   }

   if( is_update_available )
   {
     //make other slots inactive
     for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
     {
       if( slot_num != i )
       {
         cfg.slot_table[i].is_this_slot_active = 0u;
       }
     }

     uint32_t slot_addr;
     if( slot_num == 0u ) slot_addr = ETX_APP_SLOT0_FLASH_ADDR;
     else                 slot_addr = ETX_APP_SLOT1_FLASH_ADDR;

     //Load the new app or firmware to app's flash address
     ret = write_data_to_flash_app( (uint8_t*)slot_addr, cfg.slot_table[slot_num].fw_size );
     if( ret != HAL_OK )
     {
       printf("App Flash write Error\r\n");
     }
     else
     {
       /* write back the updated config */
       ret = write_cfg_to_flash( &cfg );
       if( ret != HAL_OK )
       {
         printf("Config Flash write Error\r\n");
       }
     }
   }
   else
   {
     //Find the active slot in case the update is not available
     for( uint8_t i = 0; i < ETX_NO_OF_SLOTS; i++ )
     {
       if( cfg.slot_table[i].is_this_slot_active == 1u )
       {
         slot_num = i;
         break;
       }
     }
   }

   //Verify the application is corrupted or not
   printf("Verifying the Application...");
   FLASH_WaitForLastOperation( HAL_MAX_DELAY );

   //Verify the application
   uint32_t cal_data_crc = calculate_crc32( (uint8_t*)ETX_APP_FLASH_ADDR, cfg.slot_table[slot_num].fw_size );
   FLASH_WaitForLastOperation( HAL_MAX_DELAY );

   //Verify the CRC
   if( cal_data_crc != cfg.slot_table[slot_num].fw_crc )
   {
     printf("\r\n[ERROR] App CRC Mismatch! (Cal:0x%X - Cfg:0x%X)\r\n", (unsigned int)cal_data_crc, (unsigned int)cfg.slot_table[slot_num].fw_crc);
     printf("Attempting ROLLBACK to previous version...\r\n");

     //rollback
     uint8_t backup_slot = (slot_num == 0) ? 1 : 0;

     if (cfg.slot_table[backup_slot].is_this_slot_not_valid == 0)
     {
         printf(">> Found valid Backup FW at Slot %d. Restoring...\r\n", backup_slot);
         uint32_t backup_addr = (backup_slot == 0) ? ETX_APP_SLOT0_FLASH_ADDR : ETX_APP_SLOT1_FLASH_ADDR;

         ret = write_data_to_flash_app( (uint8_t*)backup_addr, cfg.slot_table[backup_slot].fw_size );

         if (ret == HAL_OK)
         {
             printf(">> Rollback OK. Updating Config...\r\n");
             cfg.slot_table[slot_num].is_this_slot_active = 0u;
             cfg.slot_table[backup_slot].is_this_slot_active = 1u;
             write_cfg_to_flash(&cfg);

             printf(">> Resetting system...\r\n");
             HAL_Delay(100);
             HAL_NVIC_SystemReset();
         }
         else
         {
             printf(">> Rollback Failed (Flash Error). HALT.\r\n");
             while(1);
         }
     }
     else
     {
         printf(">> No valid Backup found. HALT.\r\n");
         while(1);
     }
     // end rollback
   }
   printf("Done!!!\r\n");
}

static HAL_StatusTypeDef write_cfg_to_flash( ETX_GNRL_CFG_ *cfg )
{
  HAL_StatusTypeDef ret;

  do
  {
    if( cfg == NULL )
    {
      ret = HAL_ERROR;
      break;
    }

    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );

    //Erase the Flash
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError;

    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector        = FLASH_SECTOR_4;
    EraseInitStruct.NbSectors     = 1;                    //erase only sector 4
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

    // clear all flags before you write it to flash
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

    ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
    if( ret != HAL_OK )
    {
      break;
    }

    //write the configuration
    uint8_t *data = (uint8_t *) cfg;
    for( uint32_t i = 0u; i < sizeof(ETX_GNRL_CFG_); i++ )
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_BYTE,
                               ETX_CONFIG_FLASH_ADDR + i,
                               data[i]
                             );
      if( ret != HAL_OK )
      {
        printf("Slot table Flash Write Error\r\n");
        break;
      }
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation( HAL_MAX_DELAY );

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }
  }while( false );

  return ret;
}
