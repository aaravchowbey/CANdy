#!/usr/bin/env perl

use strict;
use warnings;

open my $out, '>>', 'data/test8-poll-10x.tsv' or die "can't open!\n";

for (my $i = -5; $i <= 4; $i += 0.1) {
  my $build_flags = 'compiler.cpp.extra_flags="-D FRAME_DELAY_AMOUNT=' . (sprintf "%.2f", $i) . 'f"';
  system("make BUILD_FLAGS='$build_flags' hammer-receiver > /dev/null");

  open my $fh, '-|', 'make monitor' or die "unable to run command: $!\n";
  my @lines;
  while (my $line = <$fh>) {
    push @lines, $line;
    last if @lines == 100;
  }
  close $fh;

  shift @lines;

  my $data = sprintf("%.2f\t%s\n", "$i", scalar (grep /c0/i, @lines));

  print $data;
  print $out $data;
}
