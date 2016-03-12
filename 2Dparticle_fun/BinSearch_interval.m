function inds = BinSearch_interval( CDF, randval )
    %BINSEARCH_INTERVAL 
    
    CDF_len = length(CDF);
    randval_len = length(randval);
    inds = zeros(randval_len, 1);
    
    %for each randval, do binary search
    for i = 1:randval_len
        start = 1;
        mid = ceil(CDF_len/2);
        finish = CDF_len;
        
        while((finish - start) > 1)
            if CDF(mid) > randval(i) %look left
                %start remains the same
                finish = mid;
                mid = ceil((start+finish)/2);
            else %look right
                %finish stays the same
                start = mid;
                mid = ceil((start+finish)/2);
            end
        end
        inds(i) = finish;
    end
    
end

