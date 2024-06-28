#!/usr/bin/env perl

use strict;
use warnings;

open my $out, '>>', 'data/test10-poll-speeds-priority.tsv' or die "can't open!\n";

for (my $i = 9; $i <= 10; $i++) {
	for (1..30) {
		my $build_flags = 'compiler.cpp.extra_flags="-D SAMPLING_SPEED=' . $i . '"';
		system("make BUILD_FLAGS='$build_flags' hammer-receiver");

		open my $fh, '-|', 'make monitor' or die "unable to run command: $!\n";
		my @lines;
		while (my $line = <$fh>) {
			push @lines, $line;
			last if @lines == 100;
		}
		close $fh;

		shift @lines;

		my $data = sprintf("%d\t%d\n", "$i", scalar (grep /c0/i, @lines));

		print $data;
		print $out $data;
	}
}
