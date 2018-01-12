clear all
close all

syms th thd thdd;

xdot = [T - sin(th) - thdd;
        T - thd - sin(th)];
    
y = th;


        