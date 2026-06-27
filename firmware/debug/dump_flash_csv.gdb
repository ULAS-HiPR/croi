set pagination off
monitor halt
printf "ready=%u len=%u error=0x%x records=%u truncated=%u\n", g_flash_csv_ready, g_flash_csv_len, g_flash_csv_error, g_flash_csv_records, g_flash_csv_truncated
dump binary memory flash_log.csv (char*)&g_flash_csv_dump ((char*)&g_flash_csv_dump)+g_flash_csv_len
