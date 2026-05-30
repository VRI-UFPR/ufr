program TesteBiblioteca;
uses ufr;

var
    link: ufr_link;
    timer: ufr_link;
    res: integer;
    number: integer;

begin
    link := ufr_publisher('@new mqtt @coder msgpack @log 4');
    // timer := ufr_subscriber('@new posix:timer @time 2s');

    number := 0;
    repeat
        ufr_putln(link, '%d %d %s', 99, 100, 'Opa!!1234567890');
        number := number + 1;
    until number = 2;

    writeln('fim');
end.
