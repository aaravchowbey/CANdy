#!/usr/bin/env perl

use strict;
use warnings;

open my $out, '>>', 'data/test11-poll-speeds+offset.tsv' or die "can't open!\n";

for (my $i = 9; $i <= 10; $i++) {
	for (my $j = -5; $j <= 4; $j += 0.05) {
		for (1..30) {
			my $build_flags = 'compiler.cpp.extra_flags="-D SAMPLING_SPEED=' . $i . ' -D FRAME_DELAY_AMOUNT=' . (sprintf "%.2f", $j) . 'f"';
			system("make BUILD_FLAGS='$build_flags' hammer-receiver");

			open my $fh, '-|', 'make monitor' or die "unable to run command: $!\n";
			my @lines;
			while (my $line = <$fh>) {
				push @lines, $line;
				last if @lines == 100;
			}
			close $fh;

			shift @lines;

			my $data = sprintf("%d\t%.2f\t%d\n", "$i", "$j", scalar (grep /c0/i, @lines));

			print $data;
			print $out $data;
		}
	}
}
