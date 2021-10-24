import argparse
import mrob


parser = argparse.ArgumentParser()
parser.add_argument(
    "-output-path", 
    metavar="PATH",
    type=str,
    help="Specify path to resulting __init__.py file", 
    required=True
)
args = parser.parse_args()

with open(args.output_path, 'w') as f:
    f.write("from mrob import mrob\n\n")
    for x in dir(mrob):
        if not (len(x) > 2 and x[0] == '_' and x[1] == '_') and x.count('.') == 0:
            f.write(x + " = mrob." + x + "\n")

    f.write("\ndel(mrob)")
