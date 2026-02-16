use strict;
use warnings;
use Test::More;
use t::Util;
use File::Temp qw(tempfile tempdir);
use Digest::MD5 qw(md5_hex);
use Data::Dumper;

sub run_test {
    my $seed = shift;
    return run_prog("./t/00util/test $seed");
}

subtest "random schemes" => sub {
    my ($idx, $iters) = (0, 2000);
    while ($idx < $iters) {
        my $resp = run_test($idx);
        like $resp, "/===OK===/", "iter $idx";
        $idx++;
    }
};
        
done_testing();
