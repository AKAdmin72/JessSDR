create_project -name adc_test -dir /home/akadmin/Development/JessSDR/adc_test -pn GW2AR-LV18QN88C8/I7 -device_version C -force
add_file /home/akadmin/Development/JessSDR/adc_test/adc_test_top.v
add_file /home/akadmin/Development/JessSDR/adc_test/adc_capture.v
add_file /home/akadmin/Development/JessSDR/adc_test/trigger.v
add_file /home/akadmin/Development/JessSDR/adc_test/ring_buffer.v
add_file /home/akadmin/Development/JessSDR/adc_test/waveform_draw.v
add_file /home/akadmin/Development/JessSDR/adc_test/waveform_framebuffer.v
add_file /home/akadmin/Development/JessSDR/adc_test/line_draw.v
add_file /home/akadmin/Development/JessSDR/adc_test/ili9341_driver.v
add_file /home/akadmin/Development/JessSDR/adc_test/adc_test.cst
set_option -top_module adc_test_top
run all
