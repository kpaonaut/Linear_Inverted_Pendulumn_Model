function m = TotalMass(j)
global ulink

if j == 0
    m = 0;
else
    m = TotalMass(ulink(j).sister) + TotalMass(ulink(j).child) + ulink(j).m;
end