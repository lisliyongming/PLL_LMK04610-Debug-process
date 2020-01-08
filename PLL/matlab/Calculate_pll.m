close all;
clear all;
clc;
freq_in_doubler = 1;%if enable doubler of input
freq_in = 122.88e6;%the frequency of input clock
freq_out_ocsout = 19.2e6;%the desired frequency of OSCout output
freq_out = 122.88e6;%the desired frequency of distribured output clock channels
vco_req_max = 6175e6;%the maxmium frequency of VCO 
vco_req_min = 5870e6;%the minium frequency of VCO 
PLL2_RDIV = (1:31);%the value of input clock divider
PLL2_RDIV_DOUBLER = 2;%the doubler value for input clock
PLL2_NDIV = (1:65535);%the value of VCO N divider
PLL2_PRESCALER = (3:6);%the value of VCO prescaler
%% freq_out = (freq_in / PLL2_RDIV) * (VCO_OUT / PLL2_PRESCALER /PLL2_NDIV) / M
vco_expect_div = (ceil(vco_req_min/freq_out) : floor(vco_req_max/freq_out));
nn = 1;
mm = 1;
if freq_in_doubler == 0
    for i = 1:length(vco_expect_div)
        vco_out_present = freq_out * vco_expect_div(i);
        for r = 1:length(PLL2_RDIV)
            f1 = freq_in / PLL2_RDIV(r);
            for n = 1:length(PLL2_NDIV)
                vco_out_actual = f1 * PLL2_NDIV(n);
                result = vco_out_present / vco_out_actual;
                for pre = 1:length(PLL2_PRESCALER)
                    if ((result == PLL2_PRESCALER(pre)))
                        vco_out_actual_temp(nn) = vco_out_actual;
                        a(nn) = ceil(vco_out_actual/freq_out_ocsout)*freq_out_ocsout-vco_out_actual;
                        if(a(nn) == 0)
                            result_R(nn) = PLL2_RDIV(r);
                            result_N(nn) = PLL2_NDIV(n);
                            result_RRE(nn) = PLL2_PRESCALER(pre);
                            nn = nn + 1;
                        end
                    end
                end
            end
        end
    end
else
    for i = 1:length(vco_expect_div)
        vco_out_present = freq_out * vco_expect_div(i);
        for r = 1:length(PLL2_RDIV_DOUBLER)
            f1 = freq_in * PLL2_RDIV_DOUBLER(r);
            for n = 1:length(PLL2_NDIV)
                vco_out_actual = f1 * PLL2_NDIV(n);
                result = vco_out_present / vco_out_actual;
                for pre = 1:length(PLL2_PRESCALER)
                    if ((result == PLL2_PRESCALER(pre)))
                        vco_out_actual_temp(nn) = vco_out_actual;
                        a(nn) = ceil(vco_out_actual/freq_out_ocsout)*freq_out_ocsout-vco_out_actual;
                        if(a(nn) == 0)
                            result_R(nn) = PLL2_RDIV_DOUBLER(r);
                            result_N(nn) = PLL2_NDIV(n);
                            result_RRE(nn) = PLL2_PRESCALER(pre);
                            nn = nn + 1;
                        end
                    end
                end
            end
        end
    end
end

if nn > 1
    result_cal = [result_R;result_N;result_RRE];
    % X = [' result = ',num2str(result_cal)];
    disp('Result(R:N:PRE) is below')
    disp(result_cal);
else
    disp('NO useful parameters!')
end
