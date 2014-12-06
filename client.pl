#! /usr/bin/perl

use strict;
use warnings;

my @devices_active;

open M, '+<', '/dev/ttyACM0'
    or die "Failed to open master device: $!\n";

# Sending stuff to the master forces a full status report.
print M "hitme!\n";

while (<M>) {
  if (/^INACTIVE ([0-9]+)$/) {
    my $dev = $1;
    print "Device $dev no longer active.\n"
        if $devices_active[$dev];
    undef $devices_active[$dev];
  } elsif (/^ACTIVE ([0-9]+)\|([0-9]+)\|([^|]*)\|([^|]*)$/) {
    my ($dev, $interval, $desc, $unit) = ($1, $2, $3, $4);
    chomp($unit);
    print "Device $dev active: $interval '$desc' '$unit'.\n"
        if !$devices_active[$dev];
    $devices_active[$dev] = 1;
  } elsif (/^POLL ([0-9]+) (.*)$/) {
    my ($dev, $val) = ($1, $2);
    print "Device $dev: value $val\n";
  }
  else {
    print "Master said: $_";
  }
}
