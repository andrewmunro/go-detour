FROM golang:1.16-alpine

WORKDIR /go/build

COPY go.mod ./
COPY go.sum ./

RUN go mod download

COPY . ./

RUN go build -o /wow-nav ./cmd/wow/main.go 

WORKDIR /
RUN rm -rf /go/build

EXPOSE 8080

CMD ["/wow-nav"]