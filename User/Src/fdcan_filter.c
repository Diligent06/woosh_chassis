#include "fdcan_filter.h"

int FDCAN_filter_init(FDCAN_HandleTypeDef* hfdcan){
  FDCAN_FilterTypeDef filter;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;

  return HAL_FDCAN_ConfigFilter(hfdcan, &filter) == HAL_OK;
}

