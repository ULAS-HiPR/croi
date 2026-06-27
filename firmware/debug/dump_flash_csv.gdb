set pagination off
set confirm off
handle SIGTRAP stop noprint nopass
monitor reset halt
shell rm -f flash_log.csv
set $host_chunk = 0
set $keep_going = 1
while $keep_going
  continue
  printf "chunk=%u target_chunk=%u len=%u records=%u chunk_records=%u total_bytes=%u done=%u error=0x%x truncated=%u\n", $host_chunk, g_flash_csv_chunk_index, g_flash_csv_len, g_flash_csv_records, g_flash_csv_chunk_records, g_flash_csv_total_len, g_flash_csv_done, g_flash_csv_error, g_flash_csv_truncated
  if g_flash_csv_len > 0
    if $host_chunk == 0
      dump binary memory flash_log.csv (char*)&g_flash_csv_dump ((char*)&g_flash_csv_dump)+g_flash_csv_len
    else
      append binary memory flash_log.csv (char*)&g_flash_csv_dump ((char*)&g_flash_csv_dump)+g_flash_csv_len
    end
  end
  if g_flash_csv_done
    set $keep_going = 0
  end
  set $host_chunk = $host_chunk + 1
end
printf "wrote flash_log.csv\n"
