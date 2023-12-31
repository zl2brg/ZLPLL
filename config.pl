#! /usr/local/bin/perl

# *   Copyright (C) 2008-2022 Wanye Knowles ZL2BKC. 2023 Simon Eatough ZL2BRG
# *
# *   This program is free software; you can redistribute it and/or modify
# *   it under the terms of the GNU General Public License as published by
# *   the Free Software Foundation; either version 2 of the License, or
# *   (at your option) any later version.
# *
# *   This program is distributed in the hope that it will be useful,
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# *   GNU General Public License for more details.
# *
# *   You should have received a copy of the GNU General Public License
# *   along with this program; if not, write to the Free Software
# *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.




#
#  Generate a EEPROM iHex file with the PLL configuration parameters
#




my $xtal_freq = 10e6;
my $ref_freq = 0.5e6;

my @Freq = (1140e6,		# Chan 0
	    1139.5e6,		# Chan 1
	    1126e6,		# Chan 2
	    1125.5e6,		# Chan 3
	    1080e6,		# Chan 4
	    1079.5e6,		# Chan 5
	    1128e6,		# Chan 6
	    1139.5e6,		# Chan 7
	    1139.5e6,		# Chan 8
	    1139.5e6,		# Chan 9
	    1139.5e6,		# Chan 10
	    1139.5e6,		# Chan 11
	    1139.5e6,		# Chan 12
	    1139.5e6,		# Chan 13
	    1139.5e6,		# Chan 14
	    1139.5e6);		# Chan 15

my $data = "";

foreach $freq (@Freq) {
  $R = $xtal_freq/$ref_freq;
  $N = $freq/$ref_freq;	
  $P = 32;
  $B = int($N / $P);
  $A = $N - ($B * $P);
  
  # Perform checks to ensure we are within spec
  undef $err;
  $err = "N >= P*P-p" unless ($N >= ($P*$P)-$P);
  $err = "B >= A"    unless($B >= $A);
  $err = "B <= 8191"  unless($B <= 8191);     
  $err = "A >= 0"    unless($A >= 0);        

  print STDERR "Freq=  $freq";
  print STDERR "  **** Assert Failure: $err" if ($err);
  print STDERR "\n";

  my $Nval =  ($A) | ($B << 6);

  $data .= pack("S1L1", $R, $Nval);
}

print ihex($data);

sub ihex_line {
  my $addr = shift;
  my $type = shift;
  my $data = shift;

  my $sum = 0;
  my $l = length($data);
  $o = ":";
  $d = pack("C1n1C1", $l, $addr, $type). $data;
  foreach $H (split(//, $d)) {
    $o .= sprintf("%02X", ord($H));
    $sum += ord($H); 
  }
  $o .= sprintf("%02X\n", -$sum&0xff);
  
}

sub ihex {
  my $out = "";
  my $origin = 0;
  my $data = shift;
  my $l = length($data);

  while ($l > 0) {
    my $n = ($l > 16)?16:$l;
    
    $out .= ihex_line($origin, 0, substr($data, $origin, $n));
    $origin += $n;
    $l -= $n;
  }
  $out .= ihex_line(0, 1, "");
  
  return $out;
}

