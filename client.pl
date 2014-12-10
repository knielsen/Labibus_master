#! /usr/bin/perl

use strict;
use warnings;

use DBI;
use Time::HiRes;


my @devices_active;


my $dbh = DBI->connect("DBI:Pg:dbname=powermeter", "powermeter", undef,
                       {RaiseError=>1, AutoCommit=>1});

sub getstamp {
  my $sec_float = Time::HiRes::time();
  return int(0.5 + $sec_float*1000);
}


sub unquote {
  my ($x) = @_;
  $x =~ s/\\([0-9a-fA-F][0-9a-fA-F])/chr(hex($1))/ge;
  return $x;
}


sub device_active {
  my ($dev, $poll_interval, $description, $unit) = @_;

  my $res = $dbh->selectall_arrayref(<<SQL, undef, $dev);
SELECT active, description, unit, poll_interval
  FROM device_status
 WHERE id = ?
SQL
  # Insert a row if new status differs from existing.
  if (!scalar(@$res) ||
      !$res->[0][0] ||
      $res->[0][1] ne $description ||
      $res->[0][2] ne $unit ||
      $res->[0][3] != $poll_interval) {
    $dbh->do(<<SQL, undef, $dev, getstamp(), $description, $unit, $poll_interval);
INSERT INTO device_history VALUES (?, ?, TRUE, ?, ?, ?)
SQL
  }
}


sub device_inactive {
  my ($dev) = @_;

  my $res = $dbh->selectall_arrayref(<<SQL, undef, $dev);
SELECT active, description, unit, poll_interval
  FROM device_status
 WHERE id = ?
SQL
  # Insert an inactive row only if the device is currently listed active.
  if (scalar(@$res) && $res->[0][0]) {
    $dbh->do(<<SQL, undef, $dev, getstamp());
INSERT INTO device_history VALUES (?, ?, FALSE, NULL, NULL, NULL)
SQL
  }
}


sub
device_value {
  my ($dev, $val) = @_;
  $dbh->do(<<SQL, undef, $dev, getstamp(), $val);
INSERT INTO device_log VALUES (?, , ?)
SQL
}


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
    device_inactive($dev);
  } elsif (/^ACTIVE ([0-9]+)\|([0-9]+)\|([^|]*)\|([^|]*)$/) {
    my ($dev, $interval, $desc, $unit) = ($1, $2, unquote($3), unquote($4));
    chomp($unit);
    print "Device $dev active: $interval '$desc' '$unit'.\n"
        if !$devices_active[$dev];
    $devices_active[$dev] = 1;
    device_active($dev, $interval, $desc, $unit);
  } elsif (/^POLL ([0-9]+) (.*)$/) {
    my ($dev, $val) = ($1, $2);
    print "Device $dev: value $val\n";
    device_value($dev, $val);
  }
  else {
    print "Master said: $_";
  }
}
