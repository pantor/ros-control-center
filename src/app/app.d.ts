interface Topic {
  name: string;
  type: string;
  info?: any;
}

type Service = any;

interface Parameter {
  name: string;
  value: any;
  node?: string;
}

interface Node {
  name: string;
  topics: Topic[];
  services: Service[];
  params: Parameter[];

  publishing?: string[];
  subscribing?: string[];
}
