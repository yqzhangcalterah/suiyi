function [radio]=fmcw_radio_compute_reg_value()
        cfg.adc_freq = 20;
        cfg.fmcw_chirp_period = 50;
        cfg.fmcw_chirp_rampup = 40;
        cfg.fmcw_chirp_down = 5; 
        cfg.nchirp = 256;
        cfg.fmcw_startfreq = 78.5;
        cfg.fmcw_bandwidth = 300;
        FREQ_SYNTH_SD_RATE=400;  %in MHz
        gcd = compute_gcd(cfg.adc_freq, FREQ_SYNTH_SD_RATE);
        sd_M = FREQ_SYNTH_SD_RATE / gcd;
         total_cycle = round(cfg.fmcw_chirp_period * gcd) * sd_M;
       idle_time = cfg.fmcw_chirp_period - cfg.fmcw_chirp_rampup - cfg.fmcw_chirp_down;
        radio.nchirp = cfg.nchirp;
        radio.start_freq = DIV_RATIO(cfg.fmcw_startfreq, FREQ_SYNTH_SD_RATE);
        radio.cnt_wait = bitand(int32(log2(FREQ_SYNTH_SD_RATE * idle_time)) ,15);
        stop_freq = DIV_RATIO(cfg.fmcw_startfreq + cfg.fmcw_bandwidth * 1e-3, FREQ_SYNTH_SD_RATE);
        step_up =  bitshift(1,24) * cfg.fmcw_bandwidth / (FREQ_SYNTH_SD_RATE * cfg.fmcw_chirp_rampup * FREQ_SYNTH_SD_RATE * 8);
        bandwidth = stop_freq - radio.start_freq;
        up_cycle = ceil(1.0 * bandwidth / step_up);
        wait_cycle = bitshift(1,radio.cnt_wait); 
        down_cycle = total_cycle - up_cycle - wait_cycle;
        step_down = ceil(1.0 * bandwidth / down_cycle);
        while (step_down * (down_cycle - 1) >= bandwidth) 
                if (bandwidth < step_up * up_cycle)
                        bandwidth = step_up * up_cycle;
                else 
                        bandwidth = bandwidth+step_up;
                        up_cycle=up_cycle+1;
                end
                down_cycle = total_cycle - up_cycle - wait_cycle;
                step_down = ceil((double(bandwidth)) / (double(down_cycle)));
        end
        radio.stop_freq = radio.start_freq + bandwidth;
        radio.mid_freq = radio.start_freq + bandwidth / 2;
        radio.step_up = step_up;
        radio.step_down = step_down;
        radio.up_cycle = up_cycle;
        radio.down_cycle = down_cycle;
        radio.wait_cycle = wait_cycle;
        radio.total_cycle = total_cycle;
        
        cfg.adc_freq=20;
        cfg.rng_nfft=512;
        cfg.vel_nfft=256;
        cfg.bfm_npoint=360;
        cfg.bfm_az_left=-60;
        cfg.bfm_az_right=60;
        cfg.bfm_ev_up=60;
        cfg.bfm_ev_down=-60;
        cfg.track_fps=20;
        cfg.track_fov_az_left=-60;
        cfg.track_fov_az_right=60;
        cfg.track_near_field_thres=10;
        cfg.track_capture_delay=0.15;
        cfg.track_drop_delay=0.5;
        cfg.nvarray = 1;
        sys_params = radar_param_config(radio,cfg);   
        [k,p]=radar_param_rv2fft(sys_params,10,20);
end

function out= compute_gcd(a, b)
        if(a < b) 
                out = compute_gcd(b, a);
                return;
        end
        if(mod(a,b) == 0) 
                out= b;
                return;
        end
         compute_gcd(b, mod(a,b));
%          return;
end

function out = DIV_RATIO(val, F_SD) 
out = ((val) * 1e3 / 8 / (F_SD) - 16) * 16777216;
end


