program Assinante;
uses ufr;

var
    link: ufr_link;
    v0, v1, v2: integer;
    i: integer;
    text: string;

begin
    link := ufr_subscriber('@new mqtt @coder msgpack');

    i := 0;
    repeat 
        ufr_get(link, '> %d %d %z', @v0, @v1, @text);
        writeln(v0, ' ', v1, ' ', text);
        i := i + 1;
    until i = 10;

    ufr_close(link);
end.
